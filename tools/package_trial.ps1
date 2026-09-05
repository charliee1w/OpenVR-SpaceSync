# SPDX-License-Identifier: AGPL-3.0-only
#Requires -Version 7.0
param(
    [string]$MakeNsis = "${env:ProgramFiles(x86)}\NSIS\makensis.exe"
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$buildRoot = Join-Path $repo 'out\build\verify'
$build = Join-Path $buildRoot 'Release'
$compiler = (Resolve-Path -LiteralPath $MakeNsis).Path
$cmake = (Get-Command cmake -CommandType Application).Source

function Assert-Amd64Pe([string]$FilePath) {
    $bytes = [IO.File]::ReadAllBytes($FilePath)
    if ($bytes.Length -lt 64 -or $bytes[0] -ne 0x4D -or $bytes[1] -ne 0x5A) {
        throw "Not a PE executable: $FilePath"
    }
    $peOffset = [BitConverter]::ToInt32($bytes, 0x3C)
    if ($peOffset -lt 64 -or $peOffset -gt $bytes.Length - 24 -or
        $bytes[$peOffset] -ne 0x50 -or $bytes[$peOffset + 1] -ne 0x45 -or
        $bytes[$peOffset + 2] -ne 0 -or $bytes[$peOffset + 3] -ne 0) {
        throw "Invalid PE header: $FilePath"
    }
    if ([BitConverter]::ToUInt16($bytes, $peOffset + 4) -ne 0x8664) {
        throw "Packaged binary must target AMD64: $FilePath"
    }
}

Push-Location -LiteralPath $repo
try {
    $revision = (& git rev-parse HEAD).Trim()
    if ($LASTEXITCODE -ne 0) { throw 'Cannot identify source revision.' }
    & git diff --quiet HEAD
    if ($LASTEXITCODE -ne 0) { throw 'Commit tracked source changes before packaging a trial.' }
    $versionLine = Get-Content -LiteralPath (Join-Path $repo 'include\shared\Version.h') | Select-String '^#define SPACECAL_VERSION_STRING "([A-Za-z0-9.\-]+)"$'
    if (-not $versionLine) { throw 'Cannot identify build version.' }
    $version = $versionLine.Matches[0].Groups[1].Value
    $destination = Join-Path $repo "out\artifacts\SpaceSync-$version-$($revision.Substring(0, 7))"
    if (Test-Path -LiteralPath $destination) { throw "Artifact directory already exists: $destination" }
    $stage = Join-Path $destination 'stage'
    $bin = Join-Path $stage 'bin'
    $installer = Join-Path $stage 'dev-resources'
    New-Item -ItemType Directory -Path $bin, $installer, (Join-Path $bin 'sound') -Force | Out-Null
    # Rebuild the fixed x64 Release targets from this clean checkout. A matching
    # version string alone does not establish the source revision of old output.
    & $cmake -S $repo -B $buildRoot -G 'Visual Studio 17 2022' -A x64 *> (Join-Path $destination 'configure.log')
    if ($LASTEXITCODE -ne 0) { throw "CMake configuration failed. See $destination\configure.log" }
    & $cmake --build $buildRoot --config Release --target SpaceSync driver_spacesync --parallel 2 *> (Join-Path $destination 'build-release.log')
    if ($LASTEXITCODE -ne 0) { throw "Release build failed. See $destination\build-release.log" }
    foreach ($file in @('SpaceSync.exe', 'driver_spacesync.dll')) {
        if (-not (Test-Path -LiteralPath (Join-Path $build $file) -PathType Leaf)) { throw "Missing build output: $file" }
        Assert-Amd64Pe (Join-Path $build $file)
        $binary = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes((Join-Path $build $file)))
        if (-not $binary.Contains($version)) { throw "Build version does not match source: $file" }
    }
    Assert-Amd64Pe (Join-Path $repo '3rdparty\OpenVR\bin\win64\openvr_api.dll')
    Copy-Item -LiteralPath (Join-Path $build 'SpaceSync.exe') -Destination $bin
    Copy-Item -LiteralPath (Join-Path $repo '3rdparty\OpenVR\bin\win64\openvr_api.dll') -Destination $bin
    foreach ($file in @('LICENSE', 'LICENSE.txt', 'LICENSES', 'manifest.vrmanifest', 'icon.png')) {
        Copy-Item -LiteralPath (Join-Path $repo "resources\$file") -Destination $bin
    }
    Copy-Item -LiteralPath (Join-Path $repo 'NOTICE.md') -Destination $bin
    Get-ChildItem -LiteralPath (Join-Path $repo 'src\sound') -Filter '*.wav' -File |
        Copy-Item -Destination (Join-Path $bin 'sound')
    foreach ($file in @('installer.nsi', 'InstallLifecycle.nsh')) {
        Copy-Item -LiteralPath (Join-Path $repo "dev-resources\$file") -Destination $installer
    }
    # Only tracked driver resources enter the clean staging tree. Never package
    # stale DLLs, local captures, or a previous build left in dev-resources.
    $resources = & git -c core.quotePath=false ls-files -- dev-resources/driver
    if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate tracked driver resources.' }
    foreach ($file in $resources) {
        $target = Join-Path $stage $file
        New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force | Out-Null
        Copy-Item -LiteralPath (Join-Path $repo $file) -Destination $target
    }
    $driverBin = Join-Path $installer 'driver\bin\win64'
    New-Item -ItemType Directory -Path $driverBin -Force | Out-Null
    Copy-Item -LiteralPath (Join-Path $build 'driver_spacesync.dll') -Destination $driverBin
    Push-Location -LiteralPath $installer
    try {
        & $compiler installer.nsi *> (Join-Path $destination 'installer-build.log')
        if ($LASTEXITCODE -ne 0) { throw "NSIS compilation failed. See $destination\installer-build.log" }
    } finally { Pop-Location }
    $artifact = Join-Path $destination "SpaceSync_Installer-$version.exe"
    Copy-Item -LiteralPath (Join-Path $installer 'SpaceSync_Installer.exe') -Destination $artifact
    Copy-Item -LiteralPath (Join-Path $repo 'docs\trial-candidate.md') -Destination (Join-Path $destination 'TRIAL.md')
    Copy-Item -LiteralPath (Join-Path $repo 'tools\inspect_capture.py') -Destination $destination
    $files = @($artifact, (Join-Path $bin 'SpaceSync.exe'), (Join-Path $driverBin 'driver_spacesync.dll'), (Join-Path $bin 'openvr_api.dll'))
    $hashes = foreach ($file in $files) {
        [ordered]@{ path = [IO.Path]::GetRelativePath($destination, $file); sha256 = (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash }
    }
    $finalRevision = (& git rev-parse HEAD).Trim()
    if ($LASTEXITCODE -ne 0 -or $finalRevision -ne $revision) { throw 'Source revision changed during packaging; refusing manifest attribution.' }
    & git diff --quiet HEAD
    if ($LASTEXITCODE -ne 0) { throw 'Tracked source changed during packaging; refusing manifest attribution.' }
    [ordered]@{ version = $version; sourceRevision = $revision; builtAtUtc = [DateTime]::UtcNow.ToString('o'); files = @($hashes) } |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $destination 'manifest.json') -Encoding utf8
    Write-Output $destination
} finally { Pop-Location }
