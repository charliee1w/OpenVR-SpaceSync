# SpaceSync tracking correctness implementation plan

**Goal:** Fix the twelve findings reviewed against upstream 1.3.2 and make legacy-driver migration safe, with executable regression coverage.

**Architecture:** Keep the existing follow/override modes and calibration protocol. Publish driver configuration and alignment coherently, retain transactional calibration state, use observable subspaces for refinement, and correct transform/timing/lifecycle contracts. Extract small production helpers where needed so regression tests execute shipped logic.

**Tech stack:** C++20, OpenVR, Eigen, Win32 named pipes, CMake/CTest, NSIS.

**Spec:** The review completed in this task against upstream `be88d36ecbaa6ee6e42cb87d06c601e2229b166f`; finding IDs below retain its F1–F12 mapping.

## Constraints

- Work in fork `charliee1w/OpenVR-SpaceSync`, branch `codex/tracking-correctness`.
- Preserve existing upstream behavior except the identified defects; no streamer rewrite or new fusion mode.
- No live driver, SteamVR setting, registry-profile, or installer execution changes.
- Tests must execute production logic; record expected failures before repairs and passing results afterward.
- Keep each owner's completed work in an atomic commit. Stage explicit paths only.
- No local machine logs, calibration profiles, absolute personal paths, or private review evidence in the public fork.

## Work packages

### Driver state and transform correctness — F1, F7 ingress, F8, F9

Owner: driver worker. Files: `ServerTrackedDeviceProvider.{h,cpp}`, optional focused driver helpers, `tests/driver/`.

- [x] Add tests for mixed-generation state, disable/reset transitions, invalid history ingress, reference scale, and prediction/transform commutation.
- [x] Demonstrate failures on the original implementation.
- [x] Synchronize complete callback/configuration transactions without holding a lock over external pose acquisition; protect cross-device alignment reads.
- [x] Reject invalid tracker timing samples, including differencing across gaps, using the estimator worker's history-reset API.
- [x] Apply calibration scale to head reference and linear velocities consistently.
- [x] Run tests and commit owned paths. Coordinate `Cleanup()` with primary owner's IPC lifecycle work.

### Estimator observability, timing history, and rotation math — F6, F7 core, F10

Owner: estimator worker. Files: `AlignmentEstimator.h`, `PoseMath.h`, focused numerical helpers, `tests/estimator/`.

- [x] Turn native review probes into assertions for nullspace preservation, valid timing recovery, and mixed-axis half-turn round trips.
- [x] Demonstrate original failures.
- [x] Update observable eigenspaces while preserving prior nullspace information; inspect translation/scale coupling.
- [x] Add an explicit invalid-history boundary API, coordinated with driver worker.
- [x] Use a stable matrix-to-quaternion conversion around half-turns.
- [x] Verify controls and commit owned paths.

### Calibration transactions and profile safety — F2–F5

Owner: calibration worker. Files: `Calibration.{h,cpp}`, `Configuration.{h,cpp}`, focused overlay helpers, `tests/calibration/`.

- [x] Add native tests for replacing tracker A with B then losing a sample, parser bounds/types, solvable two-axis motion, single-axis rejection, and meaningful retry windows.
- [x] Demonstrate original failures.
- [x] Keep prior calibration coherent through cancellation/failure; tolerate bounded transient invalid samples.
- [x] Validate profile data before committing it, including complete finite chaperone quads.
- [x] Base motion observability on physical rotation/translation constraints; collect new data before retrying.
- [x] Run tests and commit owned paths.

### IPC lifecycle, installer, build integration — F11, F12, S1

Owner: primary agent. Files: `IPCServer.{h,cpp}`, driver `Cleanup()`, installer, CMake integration, `tests/ipc/`, packaging tests and documentation.

- [x] Add Windows named-pipe lifecycle tests using a unique pipe, with no live VR connection, covering empty/active/pending I/O and rapid shutdown.
- [x] Demonstrate shutdown/lifecycle failures, then make startup/stop/join/I/O ownership deterministic and keep logging alive until producers stop.
- [x] Resolve and validate runtime path before removing helpers; check registration command outcomes.
- [x] Handle leftover legacy driver by a reversible settings change in the installer, with explicit user-facing migration choice; never recursively delete arbitrary discovered driver directories.
- [x] Integrate all native regression targets in CTest; compile driver and overlay and build the installer without executing it.
- [x] Review combined diff, run required checks, document validation limits and commit.
- [x] Push verified branch to the fork and provide a reviewable comparison. Do not deploy.
