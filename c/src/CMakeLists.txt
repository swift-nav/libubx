cmake_minimum_required(VERSION 2.8.7)

set(libubx_HEADERS
  ${PROJECT_SOURCE_DIR}/include/ubx/ubx_messages.h
  ${PROJECT_SOURCE_DIR}/include/ubx/encode.h
  ${PROJECT_SOURCE_DIR}/include/ubx/decode.h
  )

add_library(ubx
  decode.c
  encode.c
  )

target_link_libraries(ubx m swiftnav)
target_include_directories(ubx PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_compile_options(ubx PRIVATE "-Wall")
target_compile_options(ubx PRIVATE "-Wextra")
target_compile_options(ubx PRIVATE "-Werror")
target_compile_options(ubx PRIVATE "-Wimplicit")
target_compile_options(ubx PRIVATE "-Wshadow")
target_compile_options(ubx PRIVATE "-Wswitch-default")
target_compile_options(ubx PRIVATE "-Wswitch-enum")
target_compile_options(ubx PRIVATE "-Wundef")
target_compile_options(ubx PRIVATE "-Wuninitialized")
target_compile_options(ubx PRIVATE "-Wpointer-arith")
target_compile_options(ubx PRIVATE "-Wcast-align")
target_compile_options(ubx PRIVATE "-Wformat=2")
target_compile_options(ubx PRIVATE "-Wimplicit-function-declaration")
target_compile_options(ubx PRIVATE "-Wredundant-decls")
target_compile_options(ubx PRIVATE "-Wformat-security")
target_compile_options(ubx PRIVATE "-fno-unwind-tables")
target_compile_options(ubx PRIVATE "-fno-asynchronous-unwind-tables")
target_compile_options(ubx PRIVATE "-std=gnu99")
target_compile_options(ubx PRIVATE "-fPIC")
# require at least gcc 5.0
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR CMAKE_C_COMPILER_VERSION VERSION_GREATER 5.0)
   target_compile_options(ubx PRIVATE "-Wfloat-conversion")
endif()

install(TARGETS ubx DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libubx_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ubx)
