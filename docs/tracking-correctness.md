# Tracking correctness fork

This is the verification record for `1.3.2-codex.1`. The subsequent
[`1.3.2-codex.2` trial candidate](trial-candidate.md) changes the timing/callback
architecture and adds independent calibration checks; the callback-cost
limitation below describes the earlier build.

This branch repairs the review findings against upstream SpaceSync 1.3.2,
commit `be88d36ecbaa6ee6e42cb87d06c601e2229b166f`. It retains the existing
Follow SLAM and override modes and IPC protocol version 12.

| Review finding | Result |
| --- | --- |
| F1: shared pose/configuration races | Serialize complete state transactions; release the state lock during external runtime queries and reject changed configurations on return. |
| F2: failed tracker replacement reuses the previous mount | Keep the previous profile and candidate identity separate; restore on cancel, timeout, or persistence failure. |
| F3: malformed chaperone data overruns storage | Validate complete quads, bounds, types, and finite numbers in a temporary profile before publishing it. |
| F4: valid two-axis calibration rejected | Use physical rotational/translation information rather than quaternion-coordinate covariance. |
| F5: retry deadline already expired | Allow full additional sampling windows, bounded history, and brief tracking loss. |
| F6: refinement forgets unobservable directions | Solve in the measured eigenspaces and preserve prior nullspace components, including coupled translation/scale bounds. |
| F7: invalid samples pollute timing history | Reject invalid tracker input and clear histories/differences across loss, gaps, and source changes while retaining learned timing. |
| F8: missing head-reference scale | Apply the same calibration scale to the raw reference and transformed device in both modes. |
| F9: position/velocity scale disagreement | Scale linear velocities consistently with device and drift transforms. |
| F10: mixed-axis half-turn conversion | Use a stable matrix-to-quaternion conversion and normalize the result. |
| F11: IPC and shutdown lifetimes | Own overlapped I/O on one worker, cancel and drain before releasing buffers, report startup failures, and drain detours before releasing trampolines/logging. |
| F12: uninstall deletes its runtime helper too soon | Resolve runtime and check deregistration before deleting the helper and driver; preserve files on failure. |
| S1: legacy driver can remain enabled | Offer reversible legacy-driver disabling during installation; preserve legacy files and profiles. |

## Build and verification

Requires Windows x64, Visual Studio 2022 C++ tools, CMake, the Vulkan SDK, and
the pinned submodules. Test builds also require Python 3 and NSIS. Normal
`build.bat` presets leave tests disabled; configure explicitly to run them:

```powershell
git submodule update --init --recursive
cmake -S . -B out/build/verify -G "Visual Studio 17 2022" -A x64 -DBUILD_TESTING=ON
cmake --build out/build/verify --config Release --parallel 4
ctest --test-dir out/build/verify -C Release --output-on-failure
```

The estimator, driver, and calibration tests execute production code. IPC tests
use disposable named pipes; installer tests use disposable files, mock runtime
executables, and the shared production NSIS lifecycle code. They never execute
the real installer or change SteamVR registration, settings, or calibration.
Individual test READMEs describe the expected failures reproduced before repair.

Verified for `1.3.2-codex.1` with MSVC 14.44, Windows SDK 10.0.26100 and
Vulkan SDK 1.4.350.0:

- Full x64 Release overlay and driver build passed.
- Release CTest: 19/19 groups passed (8.12 seconds).
- Debug CTest: 19/19 groups passed (30.13 seconds).
- Installer scenarios include six removal cases and nine migration/rollback cases.
- The actual NSIS installer compiled successfully; its Modern UI abort callback
  uses the supported custom-callback hook, avoiding duplicate callback definitions.
- Independent estimator and IPC/hook reviews completed; actionable hook findings
  were repaired and their closure checked.

## Limits

Native synthetic regressions establish the repaired contracts, not tracking
quality in a played VRChat session. No live headset session, driver deployment,
race-detector run, or hardware latency comparison is included. The populated
900-sample timing-history probe took about 1.8 ms inside a production HMD
callback on the verification machine; this is a synthetic processing cost,
and the shared lock can briefly delay other pose callbacks during that solve.

Hook tests cover the admission and rundown logic; MinHook patching and host DLL
unload still require a controlled SteamVR lifecycle test. The driver pins its
own module until the SteamVR process exits: this deliberately retains its code
for threads redirected into a detour just before hooks were disabled. Restart
SteamVR to replace or reload this driver. Physical tracker
visibility, mounting stability, base-station geometry, and transport quality
remain separate from these software fixes.
