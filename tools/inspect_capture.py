"""Inspect SpaceSync pose capture integrity and timing without accessing SteamVR.

This reports sample metadata, not absolute tracking accuracy or display latency.
The diagnostic stream may drop records to protect the runtime's pose callbacks.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

FIELDS = ("type arrival device role generation valid connected result offset "
          "qw qx qy qz px py pz vx vy vz wx wy wz ax ay az "
          "world_qw world_qx world_qy world_qz world_x world_y world_z "
          "head_qw head_qx head_qy head_qz head_x head_y head_z").split()


def inspect(source):
    if source.readline(16384).strip() != "# spacesync_pose_capture,1":
        raise ValueError("unsupported or missing capture version")
    if source.readline(16384).strip().split(",") != FIELDS:
        raise ValueError("capture schema does not match version 1")
    streams, generations = {}, set()
    records = invalid_records = total_bytes = 0
    footer = None
    while True:
        line = source.readline(16384)
        if not line:
            break
        total_bytes += len(line)
        if len(line) >= 16384 or total_bytes > 128 * 1024 * 1024:
            raise ValueError("capture exceeds bounded input size")
        if footer is not None:
            raise ValueError("data follows the capture footer")
        if line.startswith("# end,"):
            try:
                pairs = [field.split("=") for field in line.strip().split(",")[1:]]
                footer = {key: int(value) for key, value in pairs}
                if len(pairs) != 3 or set(footer) != {"accepted", "written", "dropped"}:
                    raise ValueError()
                if min(footer.values()) < 0 or footer["written"] != records or footer["accepted"] != records:
                    raise ValueError()
            except (ValueError, TypeError):
                raise ValueError("inconsistent capture footer") from None
            continue
        columns = next(csv.reader([line]))
        if len(columns) != len(FIELDS) or columns[0] != "sample":
            raise ValueError("malformed pose record")
        records += 1
        if records > 100000:
            raise ValueError("capture exceeds record limit")
        try:
            arrival = float(columns[1])
            device, role, generation, valid, connected, result = map(int, columns[2:8])
            values = list(map(float, columns[8:]))
        except ValueError:
            raise ValueError("pose record contains invalid numeric fields") from None
        if not math.isfinite(arrival) or arrival < 0 or not 0 <= device < 64 or role not in (0, 1) or generation < 0:
            raise ValueError("pose record contains invalid time or identity")
        if valid not in (0, 1) or connected not in (0, 1):
            raise ValueError("pose record contains invalid tracking flags")
        offset_ms = values[0] * 1000
        good = bool(valid and connected and result == 200 and all(map(math.isfinite, values)) and math.isfinite(offset_ms))
        for start in (9, 25, 32):
            quaternion = list(map(float, columns[start:start + 4]))
            norm2 = sum(value * value for value in quaternion)
            good = good and math.isfinite(norm2) and norm2 > 1e-12
        invalid_records += not good
        generations.add(generation)
        key = f"{'hmd' if role == 0 else 'tracker'}:{device}:{generation}"
        stream = streams.setdefault(key, {"times": [], "offsets": [], "invalid": 0, "arrival_regressions": 0})
        if stream["times"] and arrival < stream["times"][-1]:
            stream["arrival_regressions"] += 1
        stream["times"].append(arrival)
        stream["invalid"] += not good
        if math.isfinite(offset_ms):
            stream["offsets"].append(offset_ms)

    def percentile(values, fraction):
        if not values:
            return None
        ordered = sorted(values)
        position = fraction * (len(ordered) - 1)
        lower = int(position)
        weight = position - lower
        return (1 - weight) * ordered[lower] + weight * ordered[min(lower + 1, len(ordered) - 1)]

    output = {}
    for key, stream in sorted(streams.items()):
        times = sorted(stream["times"])
        intervals = [(b - a) * 1000 for a, b in zip(times, times[1:])]
        output[key] = {
            "records": len(times), "invalid": stream["invalid"],
            "duration_seconds": times[-1] - times[0],
            "arrival_regressions": stream["arrival_regressions"],
            "interval_ms_p50": percentile(intervals, .5),
            "interval_ms_p95": percentile(intervals, .95),
            "gaps_over_50ms": sum(value > 50 for value in intervals),
            "offset_ms_p50": percentile(stream["offsets"], .5),
            "offset_ms_p95": percentile(stream["offsets"], .95),
        }
    return {"records": records, "invalid_records": invalid_records, "generations": len(generations),
            "complete": footer is not None, "dropped": footer["dropped"] if footer else None, "streams": output}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--json", action="store_true", help="print machine-readable results")
    args = parser.parse_args()
    try:
        with args.capture.open(encoding="utf-8", newline="") as source:
            report = inspect(source)
    except (OSError, ValueError, OverflowError, csv.Error) as error:
        print(f"Capture could not be inspected: {error}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, allow_nan=False))
    else:
        print(f"{report['records']} records; {report['invalid_records']} invalid observations; "
              f"{report['generations']} generations; dropped: {report['dropped']}")
        if not report["complete"]:
            print("Incomplete capture: no final footer. Counts cannot establish loss coverage.")
        for name, stream in report["streams"].items():
            print(f"{name}: {stream['records']} records, {stream['invalid']} invalid, "
                  f"{stream['gaps_over_50ms']} arrival gaps over 50 ms; "
                  f"interval p95={stream['interval_ms_p95']} ms, pose offset p50={stream['offset_ms_p50']} ms")
        print("Arrival cadence and reported prediction offsets are not tracking accuracy or display latency.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
