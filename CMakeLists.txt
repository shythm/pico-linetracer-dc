cmake_minimum_required(VERSION 3.13)

# initialize the SDK directly
include(./pico-sdk/pico_sdk_init.cmake)

project(pico_linetracer_dc)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(pico_linetracer_dc
    ./src/main.c
    ./src/sensing.c
    ./src/timer.c
    ./src/motor.c
    ./src/switch.c
    ./src/oled.c
    ./src/drive.c
    ./src/fs.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico_linetracer_dc
    pico_stdlib
    hardware_adc
    hardware_pio
    hardware_pwm
    hardware_spi
)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_linetracer_dc 1)
pico_enable_stdio_uart(pico_linetracer_dc 0)

# create map/bin/hex/uf2 file in addition to ELF
pico_add_extra_outputs(pico_linetracer_dc)