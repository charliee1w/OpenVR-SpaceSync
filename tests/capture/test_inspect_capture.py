import importlib.util
import io
import json
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location("inspect_capture", Path(__file__).parents[2] / "tools" / "inspect_capture.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def sample(arrival=10, role=0, generation=7, valid=1):
    values = ["sample", arrival, role, role, generation, valid, 1, 200, -.02]
    values += [1, 0, 0, 0] + [0] * 12 + [1, 0, 0, 0] + [0] * 3 + [1, 0, 0, 0] + [0] * 3
    return ",".join(map(str, values)) + "\n"


def capture(rows, footer=True):
    result = "# spacesync_pose_capture,1\n" + ",".join(MODULE.FIELDS) + "\n" + "".join(rows)
    if footer:
        result += f"# end,accepted={len(rows)},written={len(rows)},dropped=3\n"
    return io.StringIO(result)


class InspectionTests(unittest.TestCase):
    def test_reports_gaps_invalidity_epochs_and_offsets(self):
        rows = [sample(), sample(10.01), sample(10.10, valid=0), sample(10.12, generation=8), sample(10.13, role=1)]
        result = MODULE.inspect(capture(rows))
        self.assertEqual(result["records"], 5)
        self.assertEqual(result["invalid_records"], 1)
        self.assertEqual(result["generations"], 2)
        self.assertEqual(result["streams"]["hmd:0:7"]["gaps_over_50ms"], 1)
        self.assertAlmostEqual(result["streams"]["hmd:0:7"]["offset_ms_p50"], -20)
        self.assertEqual(result["dropped"], 3)

    def test_incomplete_capture_remains_inspectable_but_is_labelled(self):
        result = MODULE.inspect(capture([sample()], footer=False))
        self.assertFalse(result["complete"])

    def test_rejects_corrupt_or_unsupported_files(self):
        for text in ["", "# spacesync_pose_capture,2\n", "# spacesync_pose_capture,1\nwrong,header\n",
                     capture([sample()]).getvalue().replace("written=1", "written=2"),
                     capture([sample()]).getvalue().replace("sample,10,", "sample,nan,"),
                     capture([sample()]).getvalue() + sample()]:
            with self.subTest(text=text), self.assertRaises(ValueError):
                MODULE.inspect(io.StringIO(text))

    def test_nonfinite_geometry_is_an_invalid_observation(self):
        row = sample().strip().split(",")
        row[13] = "nan"
        self.assertEqual(MODULE.inspect(capture([",".join(row) + "\n"]))["invalid_records"], 1)

    def test_reordered_arrivals_are_reported_and_not_negative_intervals(self):
        result = MODULE.inspect(capture([sample(10.02), sample(10), sample(10.01)]))
        stream = result["streams"]["hmd:0:7"]
        self.assertEqual(stream["arrival_regressions"], 1)
        self.assertAlmostEqual(stream["interval_ms_p50"], 10)

    def test_offset_conversion_overflow_is_invalid_and_json_remains_finite(self):
        row = sample().strip().split(",")
        row[8] = "1e308"
        report = MODULE.inspect(capture([",".join(row) + "\n"]))
        self.assertEqual(report["invalid_records"], 1)
        json.dumps(report, allow_nan=False)

    def test_unrepresentable_cadence_is_rejected(self):
        with self.assertRaises(ValueError):
            MODULE.inspect(capture([sample(0), sample(1e308)]))


if __name__ == "__main__":
    unittest.main()
