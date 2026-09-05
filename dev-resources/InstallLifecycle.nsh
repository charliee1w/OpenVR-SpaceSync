; SPDX-License-Identifier: AGPL-3.0-only
!ifndef SPACESYNC_INSTALL_LIFECYCLE
!define SPACESYNC_INSTALL_LIFECYCLE
!include "LogicLib.nsh"
!include "FileFunc.nsh"

; Shared by the installer and isolated packaging regression harness.
!macro SpaceSyncFailure MESSAGE
    DetailPrint "${MESSAGE}"
    SetErrorLevel 1
    Abort "${MESSAGE}"
!macroend

; $R7 is valid only after both the helper and the runtime tool were checked.
!macro ResolveSpaceSyncRuntime HELPER
    nsExec::ExecToStack /TIMEOUT=20000 '"${HELPER}" -openvrpath'
    Pop $R8
    Pop $R7
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "Could not resolve the SteamVR runtime. Existing files have been preserved."
    ${EndIf}
    ${If} $R7 == ""
        !insertmacro SpaceSyncFailure "SteamVR returned an empty runtime path. Existing files have been preserved."
    ${EndIf}
    ${IfNot} ${FileExists} "$R7\bin\win64\vrpathreg.exe"
        !insertmacro SpaceSyncFailure "SteamVR registration tool was not found. Existing files have been preserved."
    ${EndIf}
!macroend

!macro RemoveRegisteredSpaceSync HELPER
    !insertmacro ResolveSpaceSyncRuntime "${HELPER}"
    nsExec::ExecToStack /TIMEOUT=20000 '"$R7\bin\win64\vrpathreg.exe" removedriver "$INSTDIR\driver"'
    Pop $R8
    Pop $R9
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "SpaceSync driver deregistration failed. Existing files have been preserved."
    ${EndIf}
    nsExec::ExecToStack /TIMEOUT=20000 '"${HELPER}" -removemanifest "$INSTDIR\manifest.vrmanifest"'
    Pop $R8
    Pop $R9
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "SpaceSync manifest removal failed. Existing files have been preserved."
    ${EndIf}
    ClearErrors
    Delete "$INSTDIR\SpaceSync.exe"
    Delete "$INSTDIR\openvr_api.dll"
    RMDir /r "$INSTDIR\driver"
    ${If} ${Errors}
        !insertmacro SpaceSyncFailure "SpaceSync was deregistered, but some files could not be removed. Close SteamVR and retry."
    ${EndIf}
!macroend

; The caller owns rollback after a later installation failure. A value of 1
; means this installation changed an enabled legacy driver to disabled.
!macro MigrateLegacySpaceOverride HELPER CHANGED
    StrCpy ${CHANGED} 0
    nsExec::ExecToStack /TIMEOUT=20000 '"${HELPER}" -legacystatus'
    Pop $R8
    Pop $R9
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "Could not inspect the legacy driver. No migration was performed."
    ${EndIf}
    ${If} $R9 == "enabled"
        ${If} ${Silent}
            ${GetParameters} $R0
            ClearErrors
            ${GetOptions} $R0 "/DisableLegacy" $R1
            ${If} ${Errors}
                !insertmacro SpaceSyncFailure "An enabled SpaceOverride driver was found. Silent migration requires /DisableLegacy."
            ${EndIf}
            ${If} $R1 != ""
                !insertmacro SpaceSyncFailure "Use the exact /DisableLegacy flag to authorize legacy migration."
            ${EndIf}
        ${Else}
            MessageBox MB_YESNO|MB_ICONQUESTION|MB_DEFBUTTON2 \
                "OpenVR-SpaceOverride is installed and enabled.$\n$\nDisable its SteamVR add-on so SpaceSync can take over? Its files and calibration profiles will be preserved. You can re-enable it in SteamVR's add-on settings after removing SpaceSync.$\n$\nSteamVR must be restarted before using the new driver." \
                IDYES spacesync_legacy_consent
            !insertmacro SpaceSyncFailure "Installation cancelled; the legacy driver was left enabled."
            spacesync_legacy_consent:
        ${EndIf}
        ; Set before writing so a failed read-back also triggers restoration.
        StrCpy ${CHANGED} 1
        nsExec::ExecToStack /TIMEOUT=20000 '"${HELPER}" -disablelegacy'
        Pop $R8
        Pop $R9
        ${If} $R8 != 0
            !insertmacro SpaceSyncFailure "Could not disable the legacy driver. Installation has stopped."
        ${EndIf}
        DetailPrint "SpaceOverride disabled in SteamVR settings; its files and profiles were preserved."
    ${ElseIf} $R9 != "absent"
    ${AndIf} $R9 != "disabled"
        !insertmacro SpaceSyncFailure "The legacy driver status was not recognized. Installation has stopped."
    ${EndIf}
!macroend

!macro RestoreLegacySpaceOverride HELPER CHANGED
    ${If} ${CHANGED} == 1
        nsExec::ExecToStack /TIMEOUT=20000 '"${HELPER}" -enablelegacy'
        Pop $R8
        Pop $R9
        ${If} $R8 == 0
            StrCpy ${CHANGED} 0
            DetailPrint "Restored the previously enabled SpaceOverride driver."
        ${Else}
            DetailPrint "Could not restore SpaceOverride. Re-enable it in SteamVR's add-on settings before using the previous stack."
            SetErrorLevel 1
        ${EndIf}
    ${EndIf}
!macroend
!endif
