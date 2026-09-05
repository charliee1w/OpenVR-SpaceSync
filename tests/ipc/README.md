# IPC and shutdown regressions

`ipc_regressions` executes the production Win32 IPC server with a unique,
process-specific pipe name. Provider command methods and the log destination
are isolated; no SteamVR connection, installed driver, or live pipe is used.

Cases cover immediate startup/stop, idle listeners, one and multiple clients,
unread replies and pending reads, short-message rejection, handle counts across
restarts, concurrent/idempotent stop, same-object restart, duplicate ownership,
and invalid-endpoint startup failure. Initial native tests against the original
server failed rapid startup/stop and active-client handshakes; the idle control
passed. The replacement passes all nine transport cases.

`hook_rundown_tests` exercises the production detour admission/drain primitive.
It verifies that teardown waits through the complete callback, prevents dynamic
hook registration during shutdown, and chooses the restored target for late
entries. Failed detachment retains the trampoline path. It does not simulate
MinHook's assembly patching, DLL unload, or the SteamVR host lifecycle.

Run these from the repository's configured native test build:

```powershell
ctest --test-dir out/build/verify -C Release -R "^(ipc_|hook_)" --output-on-failure
```
