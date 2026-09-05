// SPDX-License-Identifier: AGPL-3.0-only
#include "InstallSupport.h"
#include <cstring>
#include <iostream>
#include <fstream>
#include <stdexcept>

void require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
template<class Function> void fails(Function function,const char* message) {
    try { function(); } catch(const std::runtime_error&) { return; }
    throw std::runtime_error(message);
}
struct Settings {
    bool enabled=true;
    vr::EVRSettingsError read=vr::VRSettingsError_None, write=vr::VRSettingsError_None;
    bool ignoreWrite=false;
    const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError) { return "fixture error"; }
    bool GetBool(const char* section,const char* key,vr::EVRSettingsError* error) {
        require(std::strcmp(section,"driver_spaceoverride")==0&&std::strcmp(key,"enable")==0,"wrong legacy setting");
        *error=read; return enabled;
    }
    void SetBool(const char* section,const char* key,bool value,vr::EVRSettingsError* error) {
        require(std::strcmp(section,"driver_spaceoverride")==0&&std::strcmp(key,"enable")==0,"wrong legacy setting");
        *error=write; if(write==vr::VRSettingsError_None&&!ignoreWrite) { enabled=value; read=vr::VRSettingsError_None; }
    }
};
struct Drivers {
    bool present=true, fail=false;
    uint32_t GetDriverCount() { return 1; }
    uint32_t GetDriverName(uint32_t,char* buffer,uint32_t length) {
        if(fail) return 0;
        const char* name=present?"spaceoverride":"lighthouse";
        const auto needed=static_cast<uint32_t>(std::strlen(name)+1);
        if(length>=needed) std::memcpy(buffer,name,needed);
        return needed;
    }
};
struct Applications {
    bool installed=false, autoLaunch=false;
    vr::EVRApplicationError add=vr::VRApplicationError_None, remove=vr::VRApplicationError_None, launch=vr::VRApplicationError_None;
    const char* GetApplicationsErrorNameFromEnum(vr::EVRApplicationError) { return "fixture application error"; }
    bool IsApplicationInstalled(const char*) { return installed; }
    vr::EVRApplicationError AddApplicationManifest(const char*) { if(add==vr::VRApplicationError_None) installed=true; return add; }
    vr::EVRApplicationError RemoveApplicationManifest(const char*) { if(remove==vr::VRApplicationError_None) installed=false; return remove; }
    vr::EVRApplicationError SetApplicationAutoLaunch(const char*,bool value) { if(launch==vr::VRApplicationError_None) autoLaunch=value; return launch; }
};
int main(int argc,char** argv) {
    try {
        require(argc==2,"expected disposable fixture directory");
        const auto root=std::filesystem::absolute(argv[1]);
        std::filesystem::create_directories(root / "mirrored" / "drivers" / "spaceoverride");
        std::ofstream(root / "mirrored" / "drivers" / "spaceoverride" / "driver.vrdrivermanifest")<<"fixture";
        Drivers drivers;
        require(install::hasRegisteredLegacy(drivers),"registered legacy driver missed");
        drivers.present=false;
        require(!install::hasRegisteredLegacy(drivers),"unrelated driver classified as legacy");
        require(install::hasLegacy(drivers,root / "mirrored"),"in-tree legacy mirror was missed");
        require(!install::hasLegacy(drivers,root / "empty"),"empty runtime classified as legacy");
        drivers.fail=true;
        fails([&]{install::hasRegisteredLegacy(drivers);},"driver inspection failed open");
        Settings settings;
        require(install::legacyEnabled(settings),"enabled legacy driver missed");
        settings.read=vr::VRSettingsError_UnsetSettingHasNoDefault;
        require(install::legacyEnabled(settings),"missing enable key treated as disabled");
        install::setLegacyEnabled(settings,false);
        require(!settings.enabled,"migration did not disable legacy");
        install::setLegacyEnabled(settings,true);
        require(settings.enabled,"reversible restore failed");
        settings.write=vr::VRSettingsError_WriteFailed;
        fails([&]{install::setLegacyEnabled(settings,false);},"setting write failure reported success");
        settings.write=vr::VRSettingsError_None; settings.ignoreWrite=true;
        fails([&]{install::setLegacyEnabled(settings,false);},"setting verification mismatch reported success");
        settings.read=vr::VRSettingsError_ReadFailed;
        fails([&]{install::legacyEnabled(settings);},"setting read failure treated as absent");
        Applications apps;
        apps.add=vr::VRApplicationError_InvalidManifest;
        fails([&]{install::addManifest(apps,"test.app","fixture.vrmanifest");},"manifest failure reported success");
        apps.add=vr::VRApplicationError_None; apps.launch=vr::VRApplicationError_UnknownApplication;
        fails([&]{install::addManifest(apps,"test.app","fixture.vrmanifest");},"autolaunch failure reported success");
        apps.launch=vr::VRApplicationError_None;
        install::addManifest(apps,"test.app","fixture.vrmanifest");
        require(apps.installed&&apps.autoLaunch,"manifest install did not complete");
        apps.remove=vr::VRApplicationError_InvalidManifest;
        fails([&]{install::removeManifest(apps,"test.app","fixture.vrmanifest");},"manifest removal failure reported success");
        apps.remove=vr::VRApplicationError_None;
        install::removeManifest(apps,"test.app","fixture.vrmanifest");
        require(!apps.installed,"manifest removal failed");
        install::removeManifest(apps,"test.app","fixture.vrmanifest");
        std::cout<<"PASS: installer CLI policies without runtime access\n";
        return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
