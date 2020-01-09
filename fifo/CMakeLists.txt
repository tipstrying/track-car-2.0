cmake_minimum_required (VERSION 2.6)
project (fifo)

include_directories(inc)

file( GLOB SOURCES "lib/*.cpp" "src/*.cpp" )
file( GLOB LIBSOURCES "lib/*.cpp" )
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++11")
add_library( fifo ${LIBSOURCES} )
# add_executable( xmlTest.exe ${SOURCES})
