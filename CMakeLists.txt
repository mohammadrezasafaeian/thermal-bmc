# Host tests. Builds nothing for the target - compiles the firmware's own
# sources with the host compiler so they can run on a PC.
#
#   cmake -B build && cmake --build build && ctest --test-dir build

cmake_minimum_required(VERSION 3.16)
project(thermal_bmc_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

enable_testing()

add_library(unity STATIC test/unity/unity.c)
target_include_directories(unity PUBLIC test/unity)

# warnings for our code only, not Unity's
add_library(warnings INTERFACE)
target_compile_options(warnings INTERFACE
    -Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion)

