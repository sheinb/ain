cmake_minimum_required(VERSION 3.1)

project(
  ain
  VERSION 1.0
  LANGUAGES C)

add_executable(ain src/ain.c)
find_package(TCL REQUIRED 8.6)
target_link_libraries(ain dl ${TCL_LIBRARY} rt)
include_directories( ${TCL_INCLUDE_PATH} )

