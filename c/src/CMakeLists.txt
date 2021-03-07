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
swift_set_language_standards(ubx)
swift_set_compile_options(ubx REMOVE -Wconversion)

install(TARGETS ubx DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libubx_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ubx)
