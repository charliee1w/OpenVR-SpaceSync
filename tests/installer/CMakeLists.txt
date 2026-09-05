find_package(Python3 COMPONENTS Interpreter REQUIRED)
find_program(SPACESYNC_MAKENSIS makensis HINTS "$ENV{ProgramFiles\(x86\)}/NSIS" "$ENV{ProgramFiles}/NSIS" REQUIRED)
add_executable(installer_fixture installer_fixture.cpp)
add_test(NAME installer_lifecycle COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test_lifecycle.py
    ${SPACESYNC_MAKENSIS} $<TARGET_FILE:installer_fixture>
    ${CMAKE_CURRENT_SOURCE_DIR}/lifecycle_test.nsi ${CMAKE_CURRENT_BINARY_DIR}/sandbox)
set_tests_properties(installer_lifecycle PROPERTIES TIMEOUT 60)
add_executable(installer_support_tests install_support_tests.cpp)
target_include_directories(installer_support_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../include/overlay"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/OpenVR/headers")
add_test(NAME installer_support COMMAND installer_support_tests "${CMAKE_CURRENT_BINARY_DIR}/support-sandbox")
