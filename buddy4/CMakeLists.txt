add_executable(buddy4 buddy4.c)

# pull in common dependencies
target_link_libraries(buddy4 pico_stdlib pico_time)

# enable usb output, enable uart output
pico_enable_stdio_usb(buddy4 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(buddy4)

# add url via pico_set_program_url
example_auto_set_url(buddy4)
