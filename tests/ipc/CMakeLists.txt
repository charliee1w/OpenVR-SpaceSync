add_executable(ipc_regressions ipc_regressions.cpp)
target_include_directories(ipc_regressions PRIVATE
    ${PROJECT_SOURCE_DIR}/include/driver
    ${PROJECT_SOURCE_DIR}/include/shared
    ${PROJECT_SOURCE_DIR}/3rdparty/OpenVR/headers
    ${PROJECT_SOURCE_DIR}/3rdparty/Eigen)
target_compile_features(ipc_regressions PRIVATE cxx_std_20)
target_link_libraries(ipc_regressions PRIVATE Eigen)
target_compile_definitions(ipc_regressions PRIVATE NOMINMAX)
foreach(scenario rapid empty one three pending-write short-request handles restart startup-failure)
    add_test(NAME ipc_${scenario} COMMAND ipc_regressions ${scenario})
    set_tests_properties(ipc_${scenario} PROPERTIES TIMEOUT 20)
endforeach()
add_executable(hook_rundown_tests hook_rundown_tests.cpp)
target_include_directories(hook_rundown_tests PRIVATE ${PROJECT_SOURCE_DIR}/include/driver)
target_compile_features(hook_rundown_tests PRIVATE cxx_std_20)
add_test(NAME hook_rundown COMMAND hook_rundown_tests)
set_tests_properties(hook_rundown PROPERTIES TIMEOUT 10)
