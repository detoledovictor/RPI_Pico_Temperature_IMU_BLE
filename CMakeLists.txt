# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD pico_w pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_temperature_ble_imu C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_temperature_ble_imu main.c)

target_link_libraries(pico_temperature_ble_imu pico_stdlib hardware_adc)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(pico_temperature_ble_imu pico_cyw43_arch_none)
endif()

# enable uart output, disable usb output
pico_enable_stdio_usb(pico_temperature_ble_imu 1)
pico_enable_stdio_uart(pico_temperature_ble_imu 0)


# create map/bin/hex file etc.
pico_add_extra_outputs(pico_temperature_ble_imu)

# Add the standard library to the build
target_link_libraries(pico_temperature_ble_imu
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_temperature_ble_imu PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(pico_temperature_ble_imu 
        pico_cyw43_arch_lwip_poll
        pico_lwip_mqtt
        tinyusb_board
        )
# Add the standard library to the build
target_link_libraries(pico_temperature_ble_imu
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(pico_temperature_ble_imu PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(pico_temperature_ble_imu 
        pico_cyw43_arch_lwip_poll
        pico_lwip_mqtt
        tinyusb_board
        )
