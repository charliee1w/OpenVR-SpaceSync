// SPDX-License-Identifier: AGPL-3.0-only
#pragma once

#include <openvr.h>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace install {

template<class Drivers>
bool hasRegisteredLegacy(Drivers& drivers)
{
    const uint32_t count=drivers.GetDriverCount();
    for(uint32_t i=0;i<count;++i) {
        std::vector<char> name(1024,0);
        const auto needed=drivers.GetDriverName(i,name.data(),static_cast<uint32_t>(name.size()));
        if(needed==0||needed>name.size()||name[needed-1]!='\0')
            throw std::runtime_error("Could not inspect the registered OpenVR drivers.");
        if(std::strcmp(name.data(),"spaceoverride")==0) return true;
    }
    return false;
}

template<class Drivers>
bool hasLegacy(Drivers& drivers,const std::filesystem::path& runtime)
{
    return hasRegisteredLegacy(drivers)
        || std::filesystem::is_regular_file(runtime / "drivers" / "spaceoverride" / "driver.vrdrivermanifest");
}

template<class Settings>
bool legacyEnabled(Settings& settings)
{
    vr::EVRSettingsError error=vr::VRSettingsError_None;
    const bool enabled=settings.GetBool("driver_spaceoverride",vr::k_pch_Driver_Enable_Bool,&error);
    // Drivers normally load unless explicitly disabled; an absent optional
    // enable key does not establish that the legacy driver is disabled.
    if(error==vr::VRSettingsError_UnsetSettingHasNoDefault) return true;
    if(error!=vr::VRSettingsError_None)
        throw std::runtime_error(std::string("Could not read legacy driver setting: ")+settings.GetSettingsErrorNameFromEnum(error));
    return enabled;
}

template<class Settings>
void setLegacyEnabled(Settings& settings,bool enabled)
{
    vr::EVRSettingsError error=vr::VRSettingsError_None;
    settings.SetBool("driver_spaceoverride",vr::k_pch_Driver_Enable_Bool,enabled,&error);
    if(error!=vr::VRSettingsError_None)
        throw std::runtime_error(std::string("Could not change legacy driver setting: ")+settings.GetSettingsErrorNameFromEnum(error));
    if(legacyEnabled(settings)!=enabled)
        throw std::runtime_error("Legacy driver setting did not retain the requested value.");
}

template<class Applications>
void addManifest(Applications& apps,const char* key,const char* path)
{
    auto error=apps.AddApplicationManifest(path);
    if(error!=vr::VRApplicationError_None)
        throw std::runtime_error(std::string("Could not install application manifest: ")+apps.GetApplicationsErrorNameFromEnum(error));
    error=apps.SetApplicationAutoLaunch(key,true);
    if(error!=vr::VRApplicationError_None)
        throw std::runtime_error(std::string("Could not enable application autostart: ")+apps.GetApplicationsErrorNameFromEnum(error));
}

template<class Applications>
void removeManifest(Applications& apps,const char* key,const char* path)
{
    if(!apps.IsApplicationInstalled(key)) return;
    const auto error=apps.RemoveApplicationManifest(path);
    if(error!=vr::VRApplicationError_None)
        throw std::runtime_error(std::string("Could not remove application manifest: ")+apps.GetApplicationsErrorNameFromEnum(error));
}

} // namespace install
