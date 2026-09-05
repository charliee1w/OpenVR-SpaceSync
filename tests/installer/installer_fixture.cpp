// SPDX-License-Identifier: AGPL-3.0-only
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    namespace fs = std::filesystem;
    if (argc < 2) return 1;
    const fs::path executable = fs::absolute(argv[0]);
    if (executable.filename() == "SpaceSync.exe") {
        const auto installation=executable.parent_path();
        if (std::string(argv[1]) == "-openvrpath") {
            if(fs::exists(installation / "fail-runtime")) return 9;
            if(fs::exists(installation / "empty-runtime")) return 0;
            std::cout << (executable.parent_path() / "runtime with spaces").string();
            return 0;
        }
        if (std::string(argv[1]) == "-removemanifest") return fs::exists(installation / "fail-manifest")?8:0;
        if (std::string(argv[1]) == "-legacystatus") {
            if(fs::exists(installation / "fail-inspect")) return 10;
            std::string state="absent";
            std::ifstream input(installation / "legacy-state");
            if(input) input>>state;
            std::cout<<state;
            return 0;
        }
        if (std::string(argv[1]) == "-disablelegacy") {
            std::ofstream(installation / "legacy-state")<<"disabled";
            std::ofstream(installation / "setting-written")<<"changed";
            return fs::exists(installation / "fail-disable")?11:0;
        }
        if (std::string(argv[1]) == "-enablelegacy") {
            if(fs::exists(installation / "fail-restore")) return 12;
            std::ofstream(installation / "legacy-state")<<"enabled";
            return 0;
        }
        return 1;
    }
    if (argc != 3 || std::string(argv[1]) != "removedriver") return 2;
    const auto installation = fs::path(argv[2]).parent_path();
    if (!fs::exists(installation / "SpaceSync.exe") || !fs::exists(installation / "driver")) return 3;
    if (fs::exists(installation / "fail-registration")) return 17;
    std::ofstream(installation / "deregistered") << "removed";
    return 0;
}
