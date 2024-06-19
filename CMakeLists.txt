cmake_minimum_required(VERSION 3.20)
 
# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()

include(pico_sdk_import.cmake)

project(Flight-Controller)

set(PICO_BOARD pico_w)
set(CYW43_LWIP 1)  

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

include_directories(${CMAKE_SOURCE_DIR}/include)

add_compile_options(-Wall -Wextra -Wno-comment)


# rest of your project
add_executable(firmware
    #main.cpp
    src/flight_controller.cpp
    RP2040/MPU6050/MPU6050.cpp
    RP2040/I2Cdev/I2Cdev.cpp

)


# Add pico_stdlib library which aggregates commonly used features

# create map/bin/hex/uf2 file in addition to ELF.
pico_enable_stdio_usb(firmware 1)
pico_enable_stdio_uart(firmware 0)
pico_add_extra_outputs(firmware)


target_link_libraries(firmware 
    pico_stdlib
    pico_stdio_usb
    pico_stdio
    hardware_i2c
    hardware_pwm
    hardware_adc
    pico_double
    pico_cyw43_arch_none

)

