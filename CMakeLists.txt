cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(Sensor_Task C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(Sensor_Task
    Sensor_Task.c
)

# Enable USB serial output (CDC) and disable UART0
pico_enable_stdio_usb(Sensor_Task 1)
pico_enable_stdio_uart(Sensor_Task 0)

# Link essential hardware subsystem drivers
target_link_libraries(Sensor_Task
    pico_stdlib
    hardware_i2c
    hardware_clocks
    hardware_pll
    hardware_xosc
    hardware_gpio
)

pico_add_extra_outputs(Sensor_Task)