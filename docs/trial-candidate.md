# SpaceSync 1.3.2-codex.2 trial candidate

This candidate builds on the tracking-correctness fork. The intended first trial
is Quest Pro with a rigidly mounted Lighthouse head tracker, using **HMD Driven**
(Follow SLAM). The native headset geometry stays intact while Lighthouse devices
follow the estimated relationship between spaces.

## Analysis and root causes

- Timestamps taken after locking included processing delays. Pose offsets must
  describe the same absolute target instant even when a callback has waited.
- Clock/mount solves ran under the shared pose lock. A snapshot worker moves
  expensive calculations outside callbacks, with one outstanding job and explicit
  configuration, history and reference generations for accepting its result.
- Latest-tracker extrapolation introduced error through acceleration/reversals.
  Alignment now uses bounded, bracketed historical observations matched at the
  same effective time. Displayed pose output is not buffered by this history.
- Calibration judged its fit against its own training samples. A frozen candidate
  now has to pass a separate 12-second look-around check with sufficient motion
  and coverage, positional/angular bounds, and checks on individual segments.
- Diagnostic file-close failure could terminate the host, and fallback close
  could close host stderr. Cleanup now closes only the owned log file and never
  exits SteamVR because diagnostic output failed.

The earlier repairs remain documented in [tracking-correctness.md](tracking-correctness.md).

## Validation

Native tests execute production estimator, history, provider, calibration, IPC,
installer-policy, capture and logging code. They include delayed workers,
obsolete results, missing/invalid tracking, configuration changes, shutdown,
prediction target times, independent bad calibration checks and bounded capture
backpressure. Capture tests use disposable sinks/files, and packaging tests use
disposable mock runtimes. They do not install a driver or access a live VR session.

The deterministic historical-pair comparison is documented with its trajectory,
controls and exact limitations in [PoseHistory.md](../tests/estimator/PoseHistory.md).
Its stress-case geometric residual improves from 3.264 degrees / 11.061 mm to
0.024 degrees / 0.206 mm RMS. The gentle-motion positional improvement is very
small. These are synthetic pairing errors before the drift estimator, not
measured headset accuracy, learned-delay quality or motion-to-photon latency.

The combined Windows x64 Release build passed all 30 CTest groups on 2026-09-05 (19.16 seconds). The provider group executes 29 production checks and the calibration group executes 30. Independent scoped reviews covered historical pairing, calibration validation, capture shutdown and provider timing/lifecycle; their concrete findings were repaired and regression checked. The full Debug build and all 30 CTest groups also passed (141.13 seconds). The final driver rebuild removed a Debug C runtime mismatch with MinHook; the Release driver remains on its Release runtime. Installer compilation and the exact source/binary manifest are produced by the packaging step below.

## First trial

1. Exit VRChat, SteamVR and the existing SpaceSync overlay. Do not replace a
   driver while SteamVR is running; this driver remains loaded until process exit.
2. Keep the previous installer and a copy of the existing SpaceSync installation.
   Export the existing profile before recalibration. Its Windows registry key is
   `HKCU\Software\Classes\Local Settings\Software\SpaceSync` (the `Config` value).
   Keep these backups local; they are not part of the source repository.
3. Run the candidate installer. On an existing SpaceSync installation choose
   **Reinstall**. If legacy SpaceOverride is still enabled, the installer offers
   to disable it without deleting its installation or profile.
4. Restart SteamVR. Confirm the overlay shows `1.3.2-codex.2`. Choose **HMD Driven**
   with the permanently mounted tracker. Keep your current streaming settings
   for a useful comparison.
5. Run calibration, completing both **Fit** and **Check**. Use comfortable,
   smooth turns and nods; follow any missing-motion guidance. An assumed scale
   is reported as such. A failed check keeps the prior saved calibration.
6. Before a long VRChat session, compare stationary body stability, slow and
   faster head turns, starts/stops, and a brief loss/recovery of head-tracker
   visibility. Note any whole-body jump, lag or persistent offset. Do not continue
   with a persistent displaced view or unstable tracking.

For a fair comparison, keep streamer settings, avatar/IK settings, physical
mounting and tracker visibility constant. A merge cannot repair an independently
occluded body tracker or establish whether a physical sensor is accurate.

## Optional local diagnostics

With SteamVR stopped, merge `"capturePoses": true` into the existing
`"driver_spacesync"` object in SteamVR's `steamvr.vrsettings`. Preserve the other
keys. The driver reads the option at startup and creates a unique CSV under
`%LOCALAPPDATA%\SpaceSync\captures`. No files are uploaded.

Capture is disabled by default. It records only the selected HMD/head tracker:
raw pose fields, validity, device indices, callback-entry QPC time, reported pose
offset, and alignment generation. It contains no device serials or registry
profiles. Limits are 120 seconds from the first accepted record, 100,000 records
and a 512-record queue. A busy writer drops diagnostic records instead of blocking
pose callbacks. Stop allows a 200 ms drain grace before requesting cancellation;
Windows I/O is cancelled and drained before releasing its buffers.

The file footer reports completed/dropped counts. Missing footer means incomplete
capture, for example after process termination or an I/O failure. Inspect a file:

```powershell
python -B tools/inspect_capture.py "C:\path\to\poses.csv"
python -B tools/inspect_capture.py "C:\path\to\poses.csv" --json
```

This inspector reports file integrity, invalid observations, arrival gaps and
prediction offsets. It is not an end-to-end replay of calibration/refinement and
rendered output. Set `capturePoses` back to `false` before a later SteamVR start
when no capture is needed. Delete unwanted captures manually.

## Rollback and limits

Exit SteamVR and the overlay, reinstall the previous SpaceSync build, then restore
the exported SpaceSync profile if needed. If returning to SpaceOverride, remove
SpaceSync registration using its uninstaller before enabling SpaceOverride again.
The installer preserves legacy files and profiles; it does not automatically
choose which previous stack you want to reactivate.

Historical pairing is Follow-specific. Lighthouse Driven/override retains its
existing filtered tracker-to-HMD alignment behavior, with corrected prediction
metadata. Unknown internal reference changes, optical faults, streaming jitter
and physical mount movement still require a real session to assess. Synthetic
tests and code review cannot certify every hardware/runtime interaction. The
candidate is suitable for a controlled trial only after the documented software
checks pass; a played-session result remains the final acceptance step.

## Reproducing the package

Use the build/test commands in the implementation plan, commit source changes,
then run `pwsh -File tools/package_trial.ps1`. It stages only explicit overlay
inputs and tracked driver resources, compiles NSIS without running the installer,
and writes a manifest containing the source revision and SHA-256 binary hashes.
The output directory includes the version/revision and refuses an existing name.
