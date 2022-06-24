project(TetherAPI)
cmake_minimum_required(VERSION 3.0)

find_path(ICE9_API_INCLUDE_DIR ice9.h)
find_path(FRONTPANEL_INCLUDE_DIR okFrontPanel.h)

set(LIB_SOURCES tether_api.c zip.c logger.c)
add_library(LIB_OBJECTS OBJECT ${LIB_SOURCES})
set_target_properties(LIB_OBJECTS PROPERTIES POSITION_INDEPENDENT_CODE 1)
target_compile_options(LIB_OBJECTS PRIVATE -Wall -Werror -Wno-unknown-pragmas)
target_include_directories(LIB_OBJECTS PRIVATE ${ICE9_API_INCLUDE_DIR} ${FRONTPANEL_INCLUDE_DIR})

add_library(tether_api SHARED $<TARGET_OBJECTS:LIB_OBJECTS>)
set_target_properties(tether_api PROPERTIES PUBLIC_HEADER tether_api.h)

add_library(tether_api_static STATIC  $<TARGET_OBJECTS:LIB_OBJECTS>)
set_target_properties(tether_api_static PROPERTIES PUBLIC_HEADER tether_api.h)

install(TARGETS tether_api DESTINATION lib)
install(TARGETS tether_api_static DESTINATION lib)
install(FILES tether_api.h DESTINATION include)
