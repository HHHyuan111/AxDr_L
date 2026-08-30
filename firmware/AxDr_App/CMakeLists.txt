cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME AxDr)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
project(${CMAKE_PROJECT_NAME} LANGUAGES C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

include_directories(
    "User/app"
    "User/bsp/inc"
    "User/control"
    "User/drive"
    "User/motor"
    "User/moldue/inc"
)

set(AXDR_USER_SOURCES
    User/app/axdr_app.c
    User/bsp/bsp_cpu.c
    User/bsp/bsp_fdcan.c
    User/bsp/bsp_flash.c
    User/bsp/bsp_gpio.c
    User/bsp/bsp_uart.c
    User/bsp/target_adc.c
    User/bsp/target_encoder.c
    User/bsp/target_pwm.c
    User/control/control_limit.c
    User/control/control_pid.c
    User/control/foc_svm.c
    User/control/foc_transform.c
    User/drive/drive.c
    User/moldue/display.c
    User/moldue/lcd.c
    User/moldue/vofa.c
    User/motor/alob.c
    User/motor/anticogging.c
    User/motor/calibration.c
    User/motor/eh_observer.c
    User/motor/encoder.c
    User/motor/foc_calc.c
    User/motor/foc_ctrl.c
    User/motor/foc_drv.c
    User/motor/hfsi.c
    User/motor/idpm.c
    User/motor/ipd_pos.c
    User/motor/nlob.c
    User/motor/pid.c
    User/motor/pll.c
    User/motor/pos_calc.c
    User/motor/scvm.c
    User/motor/trap_traj.c
    User/motor/util.c
)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME} ${AXDR_USER_SOURCES})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
        -O ihex
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generate HEX and print image size"
    VERBATIM
)
