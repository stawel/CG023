#arduino path see: cmake/Platform/UnixPaths
#cmake project see: https://github.com/francoiscampbell/arduino-cmake.git
set(CMAKE_TOOLCHAIN_FILE cmake/ArduinoToolchain.cmake)

cmake_minimum_required(VERSION 2.8.11)

Project(logger)

link_directories(libs)
add_subdirectory(src)
