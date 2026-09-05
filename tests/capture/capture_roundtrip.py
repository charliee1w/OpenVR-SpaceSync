"""Exercise the real native capture writer and offline reader together."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile

spec = importlib.util.spec_from_file_location("inspect_capture", Path(__file__).parents[2] / "tools" / "inspect_capture.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
with tempfile.TemporaryDirectory(prefix="spacesync-capture-") as directory:
    path = Path(directory) / "poses.csv"
    subprocess.run([sys.argv[1], "--write-fixture", str(path)], check=True, timeout=5)
    with path.open() as source:
        report = module.inspect(source)
    assert report["complete"] and report["records"] == 1 and report["invalid_records"] == 0, report
    assert report["streams"]["hmd:0:7"]["offset_ms_p50"] == -20, report
print("PASS: native capture to offline inspector")
