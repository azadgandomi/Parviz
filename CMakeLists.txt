# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/home/azad/pico/pico-sdk")

add_definitions(-DMPU6050)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Parviz C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

file(GLOB MPL_SRC "eMPL/*.c")
message("${MPL_SRC}")
add_library(mpl STATIC "${MPL_SRC}")
target_link_libraries(mpl pico_stdlib hardware_i2c)

# Add executable. Default name is the project name, version 0.1

add_executable(Parviz Parviz.cpp)
target_include_directories(Parviz PRIVATE eMPL)

pico_set_program_name(Parviz "Parviz")
pico_set_program_version(Parviz "0.1")

pico_enable_stdio_uart(Parviz 1)
pico_enable_stdio_usb(Parviz 0)

# Add the standard library to the build
target_link_libraries(Parviz pico_stdlib hardware_i2c mpl)

pico_add_extra_outputs(Parviz)

