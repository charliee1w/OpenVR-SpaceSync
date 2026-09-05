; Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md
;--------------------------------
; Include Modern UI

!include "MUI2.nsh"
!include "InstallLifecycle.nsh"

;--------------------------------
; General Configuration

!define APP_VERSION "1.3.2-codex.1"
!define APP_VERSION_META "1.3.2.1"
!define APP_NAME "SpaceSync"
!define LEGACY_APP_NAME "OpenVR-SpaceOverride"

!define INSTALL_DIR "$PROGRAMFILES64\${APP_NAME}"
!define LICENSE_FILE "../bin/LICENSE.txt"
!define FILES_DIR "../bin/"
!define DRIVER_DIR "driver"

Name "${APP_NAME}"
OutFile "${APP_NAME}_Installer.exe"
InstallDir "${INSTALL_DIR}"
InstallDirRegKey HKLM "Software\${APP_NAME}\Main" ""
RequestExecutionLevel admin
ShowInstDetails show

VIProductVersion "${APP_VERSION_META}"
VIAddVersionKey /LANG=1033 "ProductName" "${APP_NAME}"
VIAddVersionKey /LANG=1033 "FileDescription" "${APP_NAME} Installer"
VIAddVersionKey /LANG=1033 "LegalCopyright" "Copyright (c) 2026 Nyabsi. SpaceSync modifications Copyright (c) 2026 Shinyflvres. AGPL-3.0"
VIAddVersionKey /LANG=1033 "Comments" "SpaceSync is a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0), modified by Shinyflvres on 2026-08-23. See NOTICE.md."
VIAddVersionKey /LANG=1033 "FileVersion" "${APP_VERSION_META}"
VIAddVersionKey /LANG=1033 "ProductVersion" "${APP_VERSION}"

;--------------------------------
; Variables

Var alreadyInstalled
Var removeRequested
Var legacyChanged
Var registeredNew
Var manifestAttempted
Var vrRuntimePath

;--------------------------------
; Interface Settings

!define MUI_ABORTWARNING
!define MUI_CUSTOMFUNCTION_ABORT RestoreLegacyOnAbort

;--------------------------------
; Pages

!insertmacro MUI_PAGE_LICENSE "${LICENSE_FILE}"
!define MUI_PAGE_CUSTOMFUNCTION_PRE dirPre
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES

!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES

;--------------------------------
; Language

!insertmacro MUI_LANGUAGE "English"

;--------------------------------
; Functions

Function dirPre
    StrCmp $alreadyInstalled "true" 0 +2
        Abort
FunctionEnd

Function .onInit
    StrCpy $alreadyInstalled "false"
    StrCpy $removeRequested 0
    StrCpy $legacyChanged 0
    StrCpy $registeredNew 0
    StrCpy $manifestAttempted 0

    ReadRegStr $R0 HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "UninstallString"
    StrCmp $R0 "" done
    ${If} ${Silent}
        StrCpy $alreadyInstalled "true"
        Goto done
    ${EndIf}

    MessageBox MB_YESNOCANCEL|MB_ICONQUESTION \
        "${APP_NAME} is already installed.$\n$\nClick YES to Reinstall$\nClick NO to Remove$\nClick CANCEL to abort installation" \
        IDYES repair \
        IDNO remove
    Abort

    repair:
        StrCpy $alreadyInstalled "true"
        Goto done

    remove:
        StrCpy $alreadyInstalled "true"
        StrCpy $removeRequested 1

    done:
FunctionEnd

Function .onInstFailed
    StrCpy $R6 0
    ${If} $manifestAttempted == 1
        nsExec::ExecToStack /TIMEOUT=20000 '"$PLUGINSDIR\SpaceSync.exe" -removemanifest "$INSTDIR\manifest.vrmanifest"'
        Pop $R8
        Pop $R9
        ${If} $R8 != 0
            DetailPrint "Could not roll back SpaceSync application registration."
            StrCpy $R6 1
        ${Else}
            StrCpy $manifestAttempted 0
        ${EndIf}
    ${EndIf}
    ${If} $registeredNew == 1
        nsExec::ExecToStack /TIMEOUT=20000 '"$vrRuntimePath\bin\win64\vrpathreg.exe" removedriver "$INSTDIR\driver"'
        Pop $R8
        Pop $R9
        ${If} $R8 != 0
            DetailPrint "Could not roll back SpaceSync registration. Keep the legacy driver disabled until the conflicting registration is removed."
            StrCpy $R6 1
        ${Else}
            StrCpy $registeredNew 0
        ${EndIf}
    ${EndIf}
    ${If} $R6 == 0
        !insertmacro RestoreLegacySpaceOverride "$PLUGINSDIR\SpaceSync.exe" $legacyChanged
    ${Else}
        DetailPrint "Rollback is incomplete. Remove SpaceSync's failed registration before re-enabling SpaceOverride."
    ${EndIf}
    SetErrorLevel 1
FunctionEnd

Function RestoreLegacyOnAbort
    Call .onInstFailed
FunctionEnd

!macro RemoveSpaceSyncMetadata
    Delete "$INSTDIR\LICENSE.txt"
    Delete "$INSTDIR\NOTICE.md"
    Delete "$INSTDIR\LICENSE"
    Delete "$INSTDIR\LICENSES"
    Delete "$INSTDIR\manifest.vrmanifest"
    Delete "$INSTDIR\icon.png"
    Delete "$INSTDIR\sound\*.wav"
    RMDir "$INSTDIR\sound"
    DeleteRegKey HKLM "Software\${APP_NAME}"
    DeleteRegKey HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}"
    Delete "$SMPROGRAMS\${APP_NAME}.lnk"
    Delete "$INSTDIR\Uninstall.exe"
    RMDir "$INSTDIR"
!macroend


;--------------------------------
; Installer Section

Section "Install" SecInstall
    ; Stage the new helper so repair/removal does not depend on an older
    ; uninstaller that may delete its registration helper too early.
    InitPluginsDir
    SetOutPath "$PLUGINSDIR"
    File "${FILES_DIR}\SpaceSync.exe"
    File "${FILES_DIR}\openvr_api.dll"
    !insertmacro ResolveSpaceSyncRuntime "$PLUGINSDIR\SpaceSync.exe"
    StrCpy $vrRuntimePath $R7

    ${If} $removeRequested == 1
        SetOutPath "$INSTDIR"
        !insertmacro RemoveRegisteredSpaceSync "$PLUGINSDIR\SpaceSync.exe"
        SetOutPath "$TEMP"
        !insertmacro RemoveSpaceSyncMetadata
        MessageBox MB_OK "${APP_NAME} has been uninstalled."
        SetErrorLevel 0
        Quit
    ${EndIf}

    !insertmacro MigrateLegacySpaceOverride "$PLUGINSDIR\SpaceSync.exe" $legacyChanged
    ${If} $alreadyInstalled == "true"
        DetailPrint "Cleaning previous installation..."
        SetOutPath "$INSTDIR"
        !insertmacro RemoveRegisteredSpaceSync "$PLUGINSDIR\SpaceSync.exe"
        Delete "$INSTDIR\Uninstall.exe"
    ${EndIf}

    ClearErrors
    SetOutPath "$INSTDIR"

    File "${FILES_DIR}\LICENSE.txt"
    File "${FILES_DIR}\NOTICE.md"
	File "${FILES_DIR}\LICENSE"
	File "${FILES_DIR}\LICENSES"
	File "${FILES_DIR}\manifest.vrmanifest"
    File "${FILES_DIR}\SpaceSync.exe"
    File "${FILES_DIR}\openvr_api.dll"
    File "${FILES_DIR}\icon.png"
    SetOutPath "$INSTDIR\sound"
    File "${FILES_DIR}\sound\*.wav"

    SetOutPath "$INSTDIR\driver"
    File /r "${DRIVER_DIR}\*"
    ${If} ${Errors}
        !insertmacro SpaceSyncFailure "Some SpaceSync files could not be installed. Close SteamVR and retry."
    ${EndIf}

    WriteRegStr HKLM "Software\${APP_NAME}\Main" "" $INSTDIR
    WriteUninstaller "$INSTDIR\Uninstall.exe"

    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "DisplayName" "${APP_NAME}"
    WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\${APP_NAME}" "UninstallString" "$\"$INSTDIR\Uninstall.exe$\""

    CreateShortCut "$SMPROGRAMS\${APP_NAME}.lnk" "$INSTDIR\SpaceSync.exe"
    ${If} ${Errors}
        !insertmacro SpaceSyncFailure "SpaceSync installation metadata could not be written. Installation did not complete."
    ${EndIf}

    ; A timeout can follow a successful write, so uncertain registration
    ; results must also attempt rollback.
    StrCpy $registeredNew 1
    nsExec::ExecToStack /TIMEOUT=20000 '"$vrRuntimePath\bin\win64\vrpathreg.exe" adddriver "$INSTDIR\driver"'
    Pop $R8
    Pop $R9
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "SpaceSync driver registration failed. Installation did not complete."
    ${EndIf}
    SetOutPath "$INSTDIR"
    StrCpy $manifestAttempted 1
    nsExec::ExecToStack /TIMEOUT=20000 '"$INSTDIR\SpaceSync.exe" -installmanifest'
    Pop $R8
    Pop $R9
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "SpaceSync application registration failed. Installation did not complete."
    ${EndIf}
    nsExec::ExecToStack /TIMEOUT=20000 '"$INSTDIR\SpaceSync.exe" -activatemultipledrivers'
    Pop $R8
    Pop $R9
    ${If} $R8 != 0
        !insertmacro SpaceSyncFailure "SteamVR could not enable multiple drivers. Installation did not complete."
    ${EndIf}
    StrCpy $registeredNew 0
    StrCpy $manifestAttempted 0
    StrCpy $legacyChanged 0
    DetailPrint "SpaceSync installed. Restart SteamVR before using the new driver."

SectionEnd

;--------------------------------
; Uninstaller Section

Section "Uninstall"
    SetOutPath "$INSTDIR"
    !insertmacro RemoveRegisteredSpaceSync "$INSTDIR\SpaceSync.exe"
    SetOutPath "$TEMP"
    !insertmacro RemoveSpaceSyncMetadata
    DetailPrint "SpaceSync removed. Legacy SpaceOverride files/settings were not removed; re-enable its add-on in SteamVR if you want to return to it."
SectionEnd
