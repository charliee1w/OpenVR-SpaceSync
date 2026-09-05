# Native estimator and historical-pair regressions

These executables include the production headers and bundled OpenVR definitions.
They do not load OpenVR, run SteamVR, change profiles, or require tracking hardware.

Standalone Windows verification:

```powershell
cmake -S tests/estimator -B out/build/history-verify -G "Visual Studio 17 2022" -A x64
cmake --build out/build/history-verify --config Release
ctest --test-dir out/build/history-verify -C Release --output-on-failure
cmake --build out/build/history-verify --config Debug
ctest --test-dir out/build/history-verify -C Debug --output-on-failure
out/build/history-verify/Release/spacesync_history_tests.exe replay
```

The existing five `estimator_*` groups cover quaternion conversion, observable
mount refinement, discontinuous clock history, and controls. Six `history_*`
groups cover the new `align::RigidPairHistory`:

- `timing`: asynchronous source brackets, independent positive/negative lag
  signs, exact/interpolated source times and derivatives, latest common time,
  freshness, no extrapolation, invalid queries, unchanged output on failure,
  and large absolute timestamps at the supported lag extremes.
- `discontinuities`: explicit epoch reset, dropout recovery, invalidation of
  both streams, out-of-order rejection, duplicate preservation, stale snapshots.
- `invalid_quaternions`: nonfinite fields, zero quaternions, antipodal and
  mixed-axis half-turns, shortest-arc interpolation, norm overflow/underflow.
- `bounds`: capacity wrap and time-horizon eviction, with recent data retained.
- `replay`: analytic trajectories described below, evaluated through the
  production history implementation.
- `controls`: constant motion where both methods are exact, and a fixed
  mixed-axis mount rotation plus nonzero positional lever arm.

The initial five history groups were compiled against a fail-closed interface
and all failed at valid history insertion before implementation. After the
implementation all passed; the additional control group then checked the
comparison assumptions. Local red/green outputs live under
`out/build/history-verify/`, outside tracked source.

## API contract

Source timestamps use one monotonic clock: callback arrival plus the reported
pose time offset. Samples carry normalized orientation, position, and optional
motion derivatives in consistent coordinates. The caller excludes invalid
tracking and explicitly resets on recenter, changed calibration/coordinates,
or device replacement. Nonfinite samples, backwards source times, and gaps
over 100 ms invalidate both histories automatically. Duplicate timestamps are
rejected without replacing accepted data. Each epoch is identified by
`generation()` and `RigidPairMatch::generation`.

`matchLatest(latestHmdTime, tauRot, tauPos, maxLookback, match)` chooses the newest
fully available common measurement. Positive lag queries the tracker at
`hmdTime + lag`, matching `ClockAligner`. Rotation and position use separate
tracker brackets. The result retains source bracket timestamps. Endpoint
roundoff is clamped within four floating-point epsilons at the timestamp's
magnitude; it is never extrapolated. Memory is fixed at two 256-sample rings,
with an additional one-second retention horizon. Operations allocate nothing;
queries use bounded binary searches. The caller supplies synchronization.

This is a learning measurement lane. It does not buffer live output. The caller
must skip non-advancing matched times (including after lag changes), choose a
lookback policy, and reject asynchronous results whose configuration or history
epoch changed. A failed query leaves the previous output object untouched and
does not authorize using that old output as a fresh observation.

## Deterministic comparison and limits

Each replay uses 20 seconds of 90 Hz HMD and 120 Hz tracker poses. Tracker
nominal times start 1 ms later, and arrivals add `20 + 3*sin(0.7*i)` ms.
Known effective rotation/position lags are 27/44 ms. There are 1,782 matched
pairs after a 200 ms warmup. The base trajectory is:

```text
yaw(t) = 0.7*sin(7*t) + 0.18*sin(17*t) radians
p(t)   = [0.3*sin(5*t), 1.6 + 0.1*cos(9*t), 0.2*sin(11*t)] metres
```

The gentle trajectory runs the same motion at one tenth speed. Its maximum
angular speed is bounded by 0.796 rad/s and linear speed by 0.282 m/s, below
the current hard drift-confidence speed cutoffs. The faster trajectory is a
numerical stress case that includes speeds above those cutoffs.

The baseline extrapolates the latest tracker to current HMD time, using the
existing separate timing targets, perfect analytic velocity/acceleration,
acceleration gain 1, and the production 5 cm second-order displacement cap.
Historical matching compares interpolated streams at their common earlier
time. Both scenarios have a constant true space transform. The metric is
rigid-pair residual before drift confidence gating, not final rendered poses.

MSVC x64 Release results, 2026-09-05:

| Trajectory | Historical rotation RMS | Latest rotation RMS | Historical position RMS | Latest position RMS |
|---|---:|---:|---:|---:|
| Stress | 0.024351 degrees | 3.263867 degrees | 0.205859 mm | 11.061318 mm |
| Gentle | 0.000252 degrees | 0.034474 degrees | 0.002052 mm | 0.010528 mm |

Both methods are exact to numerical tolerance for the constant-motion control.
The gentle positional difference is very small. These results establish
interpolation behavior with known delays, not a measured improvement on a
Quest Pro, correct learned delays, resilience to arbitrary sensor faults, or
motion-to-photon latency. A time-varying world transform and real tracking
noise require separate validation. Known recenters should reset/rebase the
live transform immediately rather than await delayed learning.
