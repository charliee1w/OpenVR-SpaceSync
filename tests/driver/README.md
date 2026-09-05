# Driver timing and alignment regressions

The executable links the production provider, filters, IPC object and pose capture.
It does not call driver `Init`, connect to SteamVR or enable capture. Runtime pose
queries use a boundary double carrying a pose and its absolute QPC target time.
Worker tests execute the production bounded worker; provider tests wrap its real
clock/refinement solver with barriers to exercise delayed completion.

Contracts covered:

- Arrival is captured before admission/state waits. Submitted pose timestamps
  preserve their absolute target across processing, including native Follow
  output and future-predicted override queries.
- Follow learns from bracketed historical rigid pairs, including asynchronous
  sampling, acceleration and reversal, while preserving native HMD geometry and
  derivatives. Recovery needs three pairs spanning at least 25 ms.
- One outstanding solve bounds memory and backlog. Blocked solves leave body
  callbacks, settings and status responsive. Shutdown closes admission and joins
  the worker; solver exceptions are contained.
- Clock publication preserves newer input histories. Configuration changes,
  invalid poses, gaps, producer reference changes and expired results reject
  incompatible work. Mount blocks are also rejected when clock calibration
  changes, and accepted corrections pivot at the current HMD orientation.
- Invalid configuration updates preserve the previous configuration. HMD and
  calibration scale bounds are [.01,100], prediction is [0,10] frames, device
  model ratios are [1e-6,1e6], and configuration translations are bounded to
  +/-1e6 metres per component. Filter parameters must be finite and usable.

Expected failures were observed before their repairs for synchronous callback
clock solving, timestamps contaminated by state-lock waits, invalid/extreme
configuration publication, mount publication after a clock change, and tracker
reference changes below the clock's kinematic rejection threshold.

Build/run independently on Windows:

```powershell
cmake -S tests/driver -B out/build/driver-timing -G "Visual Studio 17 2022" -A x64
cmake --build out/build/driver-timing --config Release
ctest --test-dir out/build/driver-timing -C Release --output-on-failure
cmake --build out/build/driver-timing --config Debug
ctest --test-dir out/build/driver-timing -C Debug --output-on-failure
```

The printed populated-clock callback measurement measures snapshot scheduling;
it is not a live VR latency distribution. Historical alignment is Follow-only.
Override retains its existing filtered tracker-versus-HMD drift-learning path;
its output prediction timestamp is corrected, but that learner is not converted
to historical matching. Source timing still relies on driver-reported offsets
and callback arrival, not an independently verified sensor clock.
