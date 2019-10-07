cmake_minimum_required(VERSION 2.8.7)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Threads)

set(TEST_SOURCE_FILES
    check_main.c
    check_ubx.c
    )
add_executable(test_libubx ${TEST_SOURCE_FILES})

# Used to generate absolute paths in the unit tests
set(RELATIVE_PATH_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")

target_include_directories(test_libubx PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(test_libubx PRIVATE ubx ${CHECK_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_code_coverage(test_libubx AUTO ALL)

target_compile_options(test_libubx PRIVATE "-std=gnu99")

add_custom_command(
    TARGET test_libubx POST_BUILD
    COMMENT "Running unit tests"
    COMMAND test_libubx
    )
