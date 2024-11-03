cmake_minimum_required(VERSION 3.13)

set(PROGRAM_NAME pico_w_webserver)
set(PICO_BOARD pico_w)

include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

add_subdirectory(./FatFs_SPI build)

# Print the FreeRTOS Kernel Path for debugging
message(STATUS "FREERTOS_KERNEL_PATH: ${FREERTOS_KERNEL_PATH}")

project(pico_w_webserver)

pico_sdk_init()

# message("Running makefsdata python script")
# execute_process(COMMAND
#     py makefsdata.py
#     WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
# )


add_executable(${PROGRAM_NAME}
    main.c
    ntp.c
    debug.c
    hw_config.c
    buddy_1.c
)

target_compile_definitions(${PROGRAM_NAME} PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
    LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
)

target_sources(${PROGRAM_NAME} PRIVATE
    ${PICO_SDK_PATH}/lib/lwip/src/apps/sntp/sntp.c
)

target_include_directories(${PROGRAM_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../.. # for our common lwipopts
    ${CMAKE_CURRENT_LIST_DIR}/src/rp2_common/pico_lwip/include
    ${PICO_LWIP_CONTRIB_PATH}/apps/ping
)

target_link_libraries(${PROGRAM_NAME}
    # pico_cyw43_arch_lwip_threadsafe_background  # lwIP with thread safety for Pico W
    pico_lwip_http       # HTTP support for lwIP
    pico_stdlib          # Standard Pico libraries
    pico_lwip
    hardware_adc         # ADC for hardware access
    hardware_rtc
    pico_cyw43_arch_lwip_sys_freertos  # Link FreeRTOS with lwIP
    pico_lwip_iperf      # Support for iperf if needed
    FreeRTOS-Kernel-Heap4  # Link FreeRTOS kernel with heap 4
    FatFs_SPI
    hardware_pwm
    hardware_spi
)

pico_enable_stdio_usb(${PROGRAM_NAME} 1)
pico_enable_stdio_uart(${PROGRAM_NAME} 0)

pico_add_extra_outputs(${PROGRAM_NAME})