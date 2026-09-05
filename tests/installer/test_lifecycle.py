"""Run only the shared NSIS lifecycle macro, against disposable fixture files."""
import pathlib
import shutil
import subprocess
import sys
import tempfile

compiler, helper, script, output, *include_override = sys.argv[1:]
root = pathlib.Path(output).resolve()
root.mkdir(parents=True, exist_ok=True)
exe = root / "lifecycle-test.exe"
include = pathlib.Path(include_override[0]) if include_override else pathlib.Path(script).resolve().parents[2] / "dev-resources" / "InstallLifecycle.nsh"
include = include.resolve()
compiled = subprocess.run([compiler, f"/DOUTPUT={exe}", f"/DLIFECYCLE_INCLUDE={include}", script], capture_output=True, text=True)
if compiled.returncode:
    print(compiled.stdout, compiled.stderr)
    compiled.check_returncode()
for case in ("success", "missing-runtime", "runtime-failure", "empty-runtime", "registration-failure", "manifest-failure"):
    installation = pathlib.Path(tempfile.mkdtemp(prefix=case + "-", dir=root)) / "installation with spaces"
    installation.mkdir(parents=True, exist_ok=True)
    # Never execute recursive fixture removal against an unverified target.
    assert installation.resolve().is_relative_to(root)
    shutil.copyfile(helper, installation / "SpaceSync.exe")
    (installation / "openvr_api.dll").write_text("fixture", encoding="utf-8")
    (installation / "driver").mkdir(exist_ok=True)
    runtime = installation / "runtime with spaces" / "bin" / "win64"
    runtime.mkdir(parents=True, exist_ok=True)
    if case != "missing-runtime":
        shutil.copyfile(helper, runtime / "vrpathreg.exe")
    if case == "registration-failure":
        (installation / "fail-registration").touch()
    if case == "runtime-failure":
        (installation / "fail-runtime").touch()
    if case == "empty-runtime":
        (installation / "empty-runtime").touch()
    if case == "manifest-failure":
        (installation / "fail-manifest").touch()
    # NSIS treats the final /D= value as the rest of the command line; quoting
    # the whole argument prevents its special parser from recognizing it.
    command = subprocess.list2cmdline([str(exe), "/S"]) + f" /D={installation}"
    completed = subprocess.run(command, timeout=15,
                               creationflags=subprocess.CREATE_NO_WINDOW)
    if completed.returncode and (installation / "failure.txt").exists():
        print((installation / "failure.txt").read_text(errors="replace"))
    if case == "success":
        assert completed.returncode == 0, (case, completed.returncode)
        assert (installation / "deregistered").exists(), "files removed before deregistration"
        assert not (installation / "SpaceSync.exe").exists()
        assert not (installation / "driver").exists()
    else:
        assert completed.returncode != 0, "registration failure silently accepted"
        assert (installation / "SpaceSync.exe").exists(), "helper deleted despite failure"
        assert (installation / "driver").exists(), "driver deleted despite failure"
        assert not (installation / "completed").exists()
    print("PASS", case)

compiled = subprocess.run([compiler, f"/DOUTPUT={exe}", f"/DLIFECYCLE_INCLUDE={include}", "/DMIGRATION_TEST", script], capture_output=True, text=True)
if compiled.returncode:
    print(compiled.stdout, compiled.stderr)
    compiled.check_returncode()

cases = (
    ("absent", "absent", False, None, True, "absent"),
    ("already-disabled", "disabled", False, None, True, "disabled"),
    ("silent-no-consent", "enabled", False, None, False, "enabled"),
    ("silent-consent", "enabled", True, None, True, "disabled"),
    ("inspect-failure", "enabled", True, "fail-inspect", False, "enabled"),
    ("unknown-status", "unexpected", True, None, False, "unexpected"),
    ("write-failure", "enabled", True, "fail-disable", False, "enabled"),
    ("post-migration-failure", "enabled", True, "fail-after-migration", False, "enabled"),
    ("restore-failure", "enabled", True, "fail-after-migration", False, "disabled"),
)
for case, initial, consent, failure, success, expected in cases:
    installation = pathlib.Path(tempfile.mkdtemp(prefix=case + "-", dir=root)) / "installation with spaces"
    installation.mkdir(parents=True)
    assert installation.resolve().is_relative_to(root)
    shutil.copyfile(helper, installation / "SpaceSync.exe")
    (installation / "legacy-state").write_text(initial)
    (installation / "legacy-driver.dll").write_text("keep driver")
    (installation / "legacy-profile.json").write_text("keep profile")
    if failure:
        (installation / failure).touch()
    if case == "restore-failure":
        (installation / "fail-restore").touch()
    args = [str(exe), "/S"] + (["/DisableLegacy"] if consent else [])
    completed = subprocess.run(subprocess.list2cmdline(args) + f" /D={installation}", timeout=15,
                               creationflags=subprocess.CREATE_NO_WINDOW)
    assert (completed.returncode == 0) == success, (case, completed.returncode)
    assert (installation / "legacy-state").read_text() == expected, (case, "legacy state")
    assert (installation / "legacy-driver.dll").read_text() == "keep driver", case
    assert (installation / "legacy-profile.json").read_text() == "keep profile", case
    assert (installation / "completed").exists() == success, case
    if case in ("absent", "already-disabled", "silent-no-consent"):
        assert not (installation / "setting-written").exists(), case
    print("PASS", case)
