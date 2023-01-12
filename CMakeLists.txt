cmake_minimum_required(VERSION 3.13)

# initialize the SDK directly
include(./pico-sdk/pico_sdk_init.cmake)

project(pico_linetracer_dc)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(pico_linetracer_dc
    ./src/main.c
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(pico_linetracer_dc pico_stdlib)

# create map/bin/hex/uf2 file in addition to ELF
pico_add_extra_outputs(pico_linetracer_dc)