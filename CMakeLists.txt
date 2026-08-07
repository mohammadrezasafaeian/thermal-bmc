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

add_library(pid STATIC Core/Src/pid.c)
target_include_directories(pid PUBLIC Core/Inc)

add_executable(test_pid test/test_pid.c)
target_link_libraries(test_pid PRIVATE pid unity warnings m)
add_test(NAME pid COMMAND test_pid)

# Stand-ins for everything below the firmware: HAL, FreeRTOS, I2C nodes, OLED.
# test/fakes comes first on the include path, so #include "FreeRTOS.h" resolves
# to ours. PUBLIC means anything linking these inherits that ordering.
add_library(fakes STATIC
    test/fakes/fake_hal.c
    test/fakes/fake_rtos.c
    test/fakes/fake_i2c.c
    test/fakes/fake_oled.c
    test/fakes/fake_cmsis.c
    test/fakes/plant.c)
target_include_directories(fakes PUBLIC test/fakes Core/Inc)

# The firmware itself, compiled from Core/Src - not a copy.
add_library(thermal_app STATIC Core/Src/thermal_app.c)
target_link_libraries(thermal_app PUBLIC fakes pid m)

add_executable(test_nodes test/test_nodes.c)
target_link_libraries(test_nodes PRIVATE thermal_app unity warnings)
add_test(NAME nodes COMMAND test_nodes)

add_executable(test_loop test/test_loop.c)
target_link_libraries(test_loop PRIVATE thermal_app unity warnings)
add_test(NAME loop COMMAND test_loop ${CMAKE_BINARY_DIR}/sim_log.csv)
