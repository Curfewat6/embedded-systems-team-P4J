# Generated Cmake Pico project file

# cmake_minimum_required(VERSION 3.13)

# set(CMAKE_C_STANDARD 11)
# set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
#set(PICO_SDK_PATH "/home/carlk/pi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
# include(pico_sdk_import.cmake)

# project(simple_example C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
# pico_sdk_init()

#idk
add_subdirectory(./FatFs_SPI build)
# add_subdirectory(./FatFs_SPI build_FatFs_SPI)
# add for ntp - idk
# add_subdirectory(./ntp_client build_ntp_client)

# Add executable. Default name is the project name, version 0.1
add_executable(dataCapnLogINF2004 
    # simple_example.cpp
    hw_config.c
    # capturenlog.c
    # capturenlog2.c
    # capturenlog3.c
    # capturenlog4.c
    # capturenlog5.c
    # capturenlog6.c
    
    # capturenlog7.c#(working version)
    # capturenlog8.c#(testing final version)
    # capturenlog9.c
    # testkane.c
    # capturenlog10.c
    capturenlog11.c #(testing final final version)
)

# Add the standard library and FatFS/SPI to the build
target_link_libraries(dataCapnLogINF2004 
    pico_stdlib
    FatFs_SPI
    # ntp_client
    hardware_pwm
    hardware_spi    # Add this for SPI functionality
    hardware_adc
    # hardware_rtc
    # added this for ntp
    # pico_cyw43_arch_lwip_threadsafe_background
)

# pico_set_program_name(simple_example "simple_example")
# pico_set_program_version(simple_example "0.1")

# Choose source and destination for standard input and output:
#   See 4.1. Serial input and output on Raspberry Pi Pico in Getting started with Raspberry Pi Pico (https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
#   and 2.7.1. Standard Input/Output (stdio) Support in Raspberry Pi Pico C/C++ SDK (https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf):
pico_enable_stdio_uart(dataCapnLogINF2004 0)
pico_enable_stdio_usb(dataCapnLogINF2004 1)

pico_add_extra_outputs(dataCapnLogINF2004)

