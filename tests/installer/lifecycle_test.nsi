Unicode true
Name "SpaceSync isolated lifecycle regression"
OutFile "${OUTPUT}"
RequestExecutionLevel user
SilentInstall silent
!include "${LIFECYCLE_INCLUDE}"
Var legacyChanged
Function .onInit
    StrCpy $legacyChanged 0
FunctionEnd
Function .onInstFailed
!ifdef MIGRATION_TEST
    !insertmacro RestoreLegacySpaceOverride "$INSTDIR\SpaceSync.exe" $legacyChanged
!endif
    FileOpen $0 "$INSTDIR\failure.txt" w
    FileWrite $0 "runtime=$R7$\r$\nstatus=$R8$\r$\noutput=$R9$\r$\n"
    FileClose $0
FunctionEnd
Section
    SetOutPath "$INSTDIR"
!ifdef MIGRATION_TEST
    !insertmacro MigrateLegacySpaceOverride "$INSTDIR\SpaceSync.exe" $legacyChanged
    ${If} ${FileExists} "$INSTDIR\fail-after-migration"
        !insertmacro SpaceSyncFailure "Injected post-migration failure."
    ${EndIf}
!else
    !insertmacro RemoveRegisteredSpaceSync "$INSTDIR\SpaceSync.exe"
!endif
    FileOpen $0 "$INSTDIR\completed" w
    FileWrite $0 "complete"
    FileClose $0
SectionEnd
