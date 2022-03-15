cmake_minimum_required(VERSION 3.16.0)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE ZDebug)
endif()

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project("Balancing-robot-shield-firmware")

target_sources(
app PRIVATE src/main.c
)

target_include_directories(app PRIVATE src)

add_subdirectory(drivers)