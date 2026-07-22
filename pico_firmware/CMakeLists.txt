cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmak)

project(robot_pico_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico_firmware
    src/main.c
)

# Path to the prebuilt micro-ROS static lib (see README.md for how to build it)
add_subdirectory(libmicroros)

target_link_libraries(pico_firmware
    pico_stdlib
    hardware_pwm
    hardware_gpio
    hardware_i2c
    hardware_irq
    microros
)

target_include_directories(pico_firmware PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/libmicroros/include
)

pico_enable_stdio_usb(pico_firmware 0)   # USB is used for micro-ROS transport, not stdio
pico_enable_stdio_uart(pico_firmware 1)  # use UART for printf debugging instead

pico_add_extra_outputs(pico_firmware)
