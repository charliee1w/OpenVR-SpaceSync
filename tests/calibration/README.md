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

## Independent alignment check (trial candidate)

The fit stays private while a fresh 12-second look-around sequence checks it.
The check never refits the candidate or borrows fitting samples. It requires
observable motion around two axes and at least ten valid polled poses in each
of eight 1.5-second segments. Missing coverage allows up to three complete
checking sequences; contradictory measured agreement aborts even if coverage
is also insufficient. Tracking loss remains bounded at two seconds.

Both the complete sequence and each segment must meet these mismatch guards:

| Metric | Position | Orientation |
| --- | --- | --- |
| RMS | 30 mm | 2 degrees |
| 95th percentile | 50 mm | 3 degrees |
| Maximum | 100 mm | 6 degrees |

These are conservative engineering acceptance guards, not measured Quest Pro
accuracy specifications. They evaluate internal agreement of OpenVR poses
polled by the overlay, not native timestamp synchronization or external ground
truth. The UI reports the session's checked RMS agreement and whether scale
was independently measured or assumed 1.000. Reloaded or manually edited
profiles have no current check evidence. No profile schema change is needed.

Observed red controls on `cfce5f2`: `fit_remains_private`,
`heldout_bad_position`, and `heldout_bad_angle` failed because the fit was
saved before any separate motion check. A further red control,
`heldout_error_not_retried`, caught contrary evidence being discarded by a
coverage retry in the initial implementation.

`heldout_sparse_contradiction` exercises the intersection of these gates: each
run has one bad segment containing only 1 through 9 valid poses, with tracking
loss for the rest of that segment and good data elsewhere. Every nonempty
segment participates in contradiction rejection; ten poses are required only
for accepting its coverage. Sparse contrary evidence must not be retried away.

The added production tests cover those failures, good held-out completion,
per-segment failures hidden by aggregate RMS, missing axes/segments, eventual
coverage recovery, invalid-pose loss/recovery, cancellation, shutdown saves,
save-failure rollback, measured versus assumed scale, scale-corrected check
units, finite/bounded candidates, RMS/tail limits, and targeted turn/nod cues.
