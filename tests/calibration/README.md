# Calibration regression coverage

`calibration_regressions` compiles the shipped `Calibration.cpp` and
`Configuration.cpp` implementations. Test doubles replace only OpenVR accessors,
IPC, sound, and profile persistence. The tests never initialize SteamVR, connect
to a live pipe, or read/write the registry. Including the implementation files
also allows direct coverage of private solver and parser helpers without adding
test hooks to the application API.

Build standalone on Windows with MSVC:

```powershell
cmake -S tests/calibration -B out/build/calibration -G "Visual Studio 17 2022" -A x64
cmake --build out/build/calibration --config Debug
ctest --test-dir out/build/calibration -C Debug --output-on-failure
```

The main project includes the same target in CTest. Pass a case name to the
executable to run one case independently.

## Observed failures before repair

Against the original production implementations from `be88d36`:

| Case | Observed result |
| --- | --- |
| `transient_loss` | A single invalid sample aborted calibration. |
| `loss_timeout_rollback` | Tracker B retained the previous tracker A mount. |
| `two_axes_complete` | Solvable, approximately ±30-degree yaw/pitch motion was rejected. |
| `retry_wait_recover` | The retry deleted samples and aborted before new motion could be collected. |
| `parse_transaction` | Failed parsing partially replaced the active profile. |

The existing cancellation path passed its control. Two additional tests failed
while checking the transaction boundaries: `pending_identity` persisted B with
A's mount during sampling, and `save_failure_rollback` reported success after
the persistence boundary returned failure. `scale_observability` also exposed
that machine-epsilon rank detection treats float noise as independent scale
information when translation is entirely explained by rotation of a lever arm.

## Passing behavior required

The suite covers brief loss and recovery, bounded loss timeout, cancellation,
preservation of unsaved profile state, interrupted/persistence-failed saves,
two-axis completion, meaningful extra sampling followed by success, single-axis
timeout, ill-conditioned scale/mount fallback, and invalid pose ingress.

Parser cases cover invalid roots and schemas, complete relative transforms and
normalized rotations, malformed optional fields, finite/range checks, legacy
defaults, rejection without partial state changes, complete/empty chaperone
geometry, incomplete quads, oversized geometry, and invalid coordinates.

These are native regression tests of production logic with simulated hardware
inputs. They do not establish visual comfort, room-scale tracking quality, or
installer/runtime integration on a live rig.
