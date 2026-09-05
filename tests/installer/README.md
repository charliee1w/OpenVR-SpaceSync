# Installer regression fixtures

`installer_lifecycle` compiles the production NSIS lifecycle and migration
macros into a disposable, user-level fixture executable. The fixture invokes
only `installer_fixture`, copied under helper/tool names inside the test build
directory. It never launches the actual SpaceSync executable, installer,
uninstaller, SteamVR registration tool, or OpenVR runtime.

The lifecycle cases cover successful removal, missing/runtime lookup failures,
empty runtime output, deregistration failure, and manifest-removal failure.
Failures before file removal must return nonzero and preserve the payload.

The migration cases cover absent/disabled legacy drivers, explicit silent
consent, missing consent, inspection failure, unknown status, setting failure,
restoration after a later failure, and restoration failure. Legacy driver and
profile fixtures must remain untouched. Native support tests execute the CLI's
production detection, settings, and manifest policies with in-memory runtime
interfaces, including the in-tree legacy mirror.

```powershell
cmake --build out/build/verify --config Debug --target installer_fixture installer_support_tests
ctest --test-dir out/build/verify -C Debug -R '^installer_' --output-on-failure
```

The original deletion-before-deregistration macro was run through the corrected
fixture and failed the expected assertion: `files removed before deregistration`.
The repaired production macro passes. The harness accepts an optional fifth
argument naming an alternate lifecycle include for this baseline comparison.

The interactive consent dialog is compiled but not driven by these silent
fixtures. The actual product installer is compiled separately and is never run
by the test suite. No live registry profiles or SteamVR settings are changed.
