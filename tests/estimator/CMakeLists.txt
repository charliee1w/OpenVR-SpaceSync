cmake_minimum_required(VERSION 3.20)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(SpaceSyncEstimatorTests LANGUAGES CXX)
    enable_testing()
endif()

add_executable(spacesync_estimator_tests EstimatorTests.cpp)
target_compile_features(spacesync_estimator_tests PRIVATE cxx_std_20)
if(MSVC)
    target_compile_definitions(spacesync_estimator_tests PRIVATE
        _HAS_DEPRECATED_NEGATORS=1 _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
endif()
target_include_directories(spacesync_estimator_tests PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../include/driver"
    "${CMAKE_CURRENT_LIST_DIR}/../../3rdparty/OpenVR/headers"
    "${CMAKE_CURRENT_LIST_DIR}/../../3rdparty/Eigen")
foreach(case_name IN ITEMS quaternion rotation_refinement translation_refinement clock_history controls)
    add_test(NAME estimator_${case_name} COMMAND spacesync_estimator_tests ${case_name})
endforeach()

add_executable(spacesync_history_tests PoseHistoryTests.cpp)
target_compile_features(spacesync_history_tests PRIVATE cxx_std_20)
target_include_directories(spacesync_history_tests PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../include/driver"
    "${CMAKE_CURRENT_LIST_DIR}/../../3rdparty/OpenVR/headers")
foreach(case_name IN ITEMS timing discontinuities invalid_quaternions bounds replay controls)
    add_test(NAME history_${case_name} COMMAND spacesync_history_tests ${case_name})
endforeach()
