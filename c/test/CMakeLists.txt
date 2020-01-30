cmake_minimum_required(VERSION 2.8.7)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Threads)

set(TEST_SOURCE_FILES
    check_main.c
    check_ubx.c
    )

swift_add_test(test_libubx
        POST_BUILD
        SRCS ${TEST_SOURCE_FILES}
        LINK ubx check Threads::Threads
        )

target_compile_options(test_libubx PRIVATE "-std=gnu99")
