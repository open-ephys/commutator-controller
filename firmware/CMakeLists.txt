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
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(commutator C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(commutator main.cpp ltc4425.cpp is32fl3193.cpp tmc2130.cpp parser.cpp accelstepper/AccelStepper.cpp tinyjson/tiny-json.c)

pico_set_program_name(commutator "open-ephys-commutator")
pico_set_program_version(commutator "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(commutator 0)
pico_enable_stdio_usb(commutator 1)
 
# Add the standard library to the build
target_link_libraries(commutator PRIVATE
    pico_stdlib
    pico_time
    pico_multicore
    hardware_adc 
    hardware_i2c 
    hardware_spi
    hardware_pio
    hardware_dma)

# Add the standard include files to the build
target_include_directories(commutator PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
   accelstepper 
   tinyjson
)

# target_sources(commutator PRIVATE 
#     accelstepper/accelstepper.cpp
# )

# # Compile the PIO-programs and include it into the project
# pico_add_extra_outputs(commutator)
# pico_generate_pio_header(commutator ${CMAKE_CURRENT_LIST_DIR}/picostepper/driver/four_wire.pio)
# pico_generate_pio_header(commutator ${CMAKE_CURRENT_LIST_DIR}/picostepper/driver/two_wire.pio)


