cmake_minimum_required(VERSION 2.8.7)
project(libubx)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake" "${CMAKE_CURRENT_LIST_DIR}/cmake/common")

include(GNUInstallDirs)
include(CCache)
include(SwiftCmakeOptions)

swift_create_project_options(
    HAS_TESTS
    TEST_PACKAGES "Check"
    )

find_package(Swiftnav REQUIRED)

include(ClangFormat)
swift_setup_clang_format()

include(ClangTidy)
swift_setup_clang_tidy(PATTERNS "src/*.c")

add_subdirectory (src)

if(libubx_BUILD_TESTS)
  add_subdirectory (test)
endif()

