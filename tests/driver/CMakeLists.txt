cmake_minimum_required(VERSION 3.20)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(SpaceSyncDriverTests LANGUAGES CXX)
    enable_testing()
endif()
get_filename_component(SPACESYNC_DRIVER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
add_executable(spacesync_driver_tests
    driver_tests.cpp
    boundary_stubs.cpp
    "${SPACESYNC_DRIVER_ROOT}/src/driver/ServerTrackedDeviceProvider.cpp"
    "${SPACESYNC_DRIVER_ROOT}/src/driver/KalmanFilter.cpp"
    "${SPACESYNC_DRIVER_ROOT}/src/driver/IPCServer.cpp")
target_compile_features(spacesync_driver_tests PRIVATE cxx_std_20)
target_include_directories(spacesync_driver_tests PRIVATE
    "${SPACESYNC_DRIVER_ROOT}/include/driver"
    "${SPACESYNC_DRIVER_ROOT}/include/shared"
    "${SPACESYNC_DRIVER_ROOT}/3rdparty/OpenVR/headers"
    "${SPACESYNC_DRIVER_ROOT}/3rdparty/Eigen")
target_compile_definitions(spacesync_driver_tests PRIVATE NOMINMAX
    _HAS_DEPRECATED_NEGATORS=1 _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
add_test(NAME driver_state_and_transforms COMMAND spacesync_driver_tests)
set_tests_properties(driver_state_and_transforms PROPERTIES TIMEOUT 30)
