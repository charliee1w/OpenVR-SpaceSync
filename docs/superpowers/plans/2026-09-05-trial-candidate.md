# SpaceSync validated trial candidate implementation plan

**Goal:** Prepare a reproducible Windows x64 candidate with bounded pose processing, correctly timed historical alignment, independently validated calibration, and useful trial diagnostics.

**Architecture:** Keep native Follow SLAM output and the existing override option. Capture pose arrival time before contention, pair historical observations at a common effective time, and solve expensive alignment work outside pose callbacks. Publish only results from the current configuration/reference generation. Keep saved calibration transactional until a separate validation movement succeeds.

**Tech stack:** C++20, OpenVR, Eigen, Win32, CMake/CTest, Python, NSIS.

**Spec:** The architecture assessment in this task against `cfce5f2`, authorized by the user to implement and validate before their first trial.

## Constraints and acceptance

- Preserve previous fixes and completed atomic commits on `codex/tracking-correctness`.
- No live installation, SteamVR interruption, hardware configuration, or profile changes while developing and validating.
- Preserve native device output timing; bounded historical learning must not introduce a pose-output buffer.
- Bound memory, queued work and capture files; drain worker lifetimes before shutdown.
- Never publish a background result from an obsolete configuration or reference frame.
- Reject unsupported, nonfinite, stale or cross-gap measurements; do not extrapolate history across missing evidence.
- A calibration fits and validates on distinct motion segments; failed validation retains the prior profile.
- Software/replay evidence does not establish physical tracking quality or motion-to-photon latency.
- Experimental filter identification, additional hardware anchors, and streamer integrations are outside this candidate.

## Work packages

### Driver timing and worker lifecycle

Owner: driver worker. Files: `ServerTrackedDeviceProvider.{h,cpp}`, focused worker helpers, `tests/driver/`.

- [x] Reproduce timestamp contention and callback solve costs with production-path tests.
- [x] Capture arrival before locks and retain explicit prediction target semantics.
- [x] Move expensive solves to bounded background work; publish coherent current-generation results.
- [x] Integrate timestamped history; invalidate incompatible epochs and support immediate known recenter handling.
- [x] Verify delayed solves/configuration changes, shutdown, burst input, invalid poses and prediction timing; commit the tested unit.

### Historical pairing and deterministic replay

Owner: estimator worker. Files: new `PoseHistory.h`, `tests/estimator/`, focused estimator headers if required.

- [x] Add failing tests for asynchronous pose streams, independent lag signs and motion reversals.
- [x] Implement bounded quaternion/position interpolation with explicit generation, freshness and gap checks.
- [x] Cover nonfinite, duplicate/out-of-order input, capacity wrap, resets and dropout recovery.
- [x] Compare historical pairing with latest-pose extrapolation on known deterministic trajectories; document the limits and commit.

### Calibration validation and guidance

Owner: calibration worker. Files: `Calibration.{h,cpp}`, `UserInterface.{h,cpp}`, `tests/calibration/`.

- [x] Add production regressions for good fit/bad independent validation and profile preservation.
- [x] Retain fitted candidate privately while gathering a separate validation movement.
- [x] Require adequate independent motion and positional/angular consistency before publication.
- [x] Give specific missing-motion guidance and report scale measured versus assumed.
- [x] Verify success, rejection, insufficient coverage, tracking loss, cancel, timeout and persistence failure; commit.

### Trial diagnostics and delivery

Owner: primary agent. Files: new bounded pose-capture helper/tests, diagnostic tool, root CMake, version, release documentation and artifacts.

- [x] Implement opt-in local pose capture with nonblocking ingress, background writes, explicit time/epoch/role metadata and hard duration/size limits.
- [x] Verify disabled mode, malformed input, queue saturation, startup/write failures, shutdown and bounded files.
- [x] Integrate capture without logging private profiles or device serials; provide an offline inspection tool.
- [x] Review cross-component timing, reference and lifetime contracts; repair concrete review findings.
- [x] Build full x64 Release overlay/driver and installer, run all CTest groups in Release and Debug, inspect the packaged file set.
- [x] Produce a source patch, checksummed trial installer and clear install/rollback/trial instructions. Commit and update the existing fork branch.

## Verification commands

```powershell
cmake -S . -B out/build/verify -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTING=ON
cmake --build out/build/verify --config Release --parallel 4
ctest --test-dir out/build/verify -C Release --output-on-failure
cmake --build out/build/verify --config Debug --parallel 4
ctest --test-dir out/build/verify -C Debug --output-on-failure
```

Each owner records failing regression evidence and focused green results before committing. Final evidence and practical limitations belong in `docs/trial-candidate.md`; installer compilation is not installer execution or hardware acceptance.
