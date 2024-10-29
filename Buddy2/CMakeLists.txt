add_subdirectory(draft)

if (TARGET tinyusb_device)
    add_executable(project_buddy2
            project_buddy2.c
            )

    # pull in common dependencies
    target_link_libraries(project_buddy2 
            pico_stdlib
            pico_cyw43_arch_none
            
            hardware_gpio
            hardware_pwm
            hardware_adc
            )

    target_include_directories(project_buddy2 PRIVATE
            ${CMAKE_CURRENT_LIST_DIR}
            )
    

    # enable usb output, disable uart output
    pico_enable_stdio_usb(project_buddy2 1)
    pico_enable_stdio_uart(project_buddy2 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(project_buddy2)

elseif(PICO_ON_DEVICE)
    message(WARNING "not building hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
