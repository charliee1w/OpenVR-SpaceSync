cmake_minimum_required(VERSION 3.20)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(SpaceSyncCalibrationTests LANGUAGES CXX)
    enable_testing()
endif()

get_filename_component(SPACESYNC_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
add_executable(calibration_regressions calibration_regressions.cpp)
target_compile_features(calibration_regressions PRIVATE cxx_std_20)
target_compile_definitions(calibration_regressions PRIVATE
    _HAS_DEPRECATED_NEGATORS=1 _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
target_include_directories(calibration_regressions PRIVATE
    "${SPACESYNC_ROOT}/include/overlay"
    "${SPACESYNC_ROOT}/include/shared"
    "${SPACESYNC_ROOT}/3rdparty/Eigen"
    "${SPACESYNC_ROOT}/3rdparty/PicoJSON"
    "${SPACESYNC_ROOT}/3rdparty/OpenVR/headers")
target_link_libraries(calibration_regressions PRIVATE advapi32)
if(MSVC)
    target_compile_options(calibration_regressions PRIVATE /bigobj)
endif()
add_test(NAME calibration_regressions COMMAND calibration_regressions)
