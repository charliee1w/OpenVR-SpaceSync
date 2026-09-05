# Estimator regression tests

These native tests include the production driver headers and the bundled OpenVR
headers and Eigen implementation. They do not load a driver, connect to SteamVR,
or use replacement OpenVR types.

Run through the repository's CTest build, or build this subdirectory alone:

```powershell
cmake -S tests/estimator -B out/estimator -G "Visual Studio 17 2022" -A x64
cmake --build out/estimator --config Release
ctest --test-dir out/estimator -C Release --output-on-failure
```

- `quaternion`: exact mixed-axis half-turns, float matrices near a half-turn,
  ordinary rotations, unit length, and nonfinite input fallback.
- `rotation_refinement`: retain an earlier correction in the unobservable
  direction of a fixed tilted pose while accepting an observable correction,
  including total and per-step bounds.
- `translation_refinement`: retain the unobservable lever arm under single-axis
  motion and the coupled translation/scale nullspace under rich rotational
  motion, including bounds and coupled step limits.
- `clock_history`: preserve the last learned timing estimate while excluding
  samples across explicit invalid intervals and missing-pose gaps; reject a
  short recovery window, then learn from sufficient valid recovery samples.
- `controls`: exact rebase mapping, empty and stationary blocks, fully observable
  mount/scale recovery, fixed scale, nonfinite solve rejection, and stationary
  clock rejection.

The initial tests were run against the reviewed implementation before repairs.
Quaternion conversion, rotation refinement, translation refinement, and HMD
history-boundary tests failed for their asserted defects; the original controls
passed. After repair, all five groups pass with MSVC Release builds.

The timing and refinement fixtures are synthetic mathematical regressions. Their
injected timing offsets and correction errors are not measurements of a headset
or a played VR session.
