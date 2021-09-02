cmake_minimum_required(VERSION 2.8.7)

set(libubx_HEADERS
  ${PROJECT_SOURCE_DIR}/include/ubx/ubx_messages.h
  ${PROJECT_SOURCE_DIR}/include/ubx/encode.h
  ${PROJECT_SOURCE_DIR}/include/ubx/decode.h
  )

swift_add_library(ubx
  SOURCES
    decode.c
    encode.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
add_library(swiftnav::ubx ALIAS ubx)

target_link_libraries(ubx PUBLIC swiftnav)
target_include_directories(ubx PUBLIC ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS ubx DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libubx_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ubx)
