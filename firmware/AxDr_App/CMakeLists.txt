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
    "User/common"
    "User/control"
    "User/drive"
    "User/motor"
    "User/moldue/inc"
)

set(AXDR_USER_SOURCES
    User/app/control_cycle.c
    User/app/fast_loop.c
    User/app/debug_snapshot.c
    User/bsp/bsp_cpu.c
    User/bsp/bsp_fdcan.c
    User/bsp/bsp_flash.c
    User/bsp/bsp_gpio.c
    User/bsp/bsp_uart.c
    User/bsp/target_adc.c
    User/bsp/target_encoder.c
    User/bsp/target_pwm.c
    User/control/control_cascade.c
    User/control/control_filter.c
    User/control/control_limit.c
    User/control/control_loop.c
    User/control/control_mit.c
    User/control/control_pid.c
    User/control/control_speed.c
    User/control/foc_core.c
    User/control/foc_svm.c
    User/control/foc_transform.c
    User/drive/drive.c
    User/drive/drive_mode.c
    User/drive/drive_protection.c
    User/drive/drive_pwm.c
    User/moldue/display.c
    User/moldue/lcd.c
    User/moldue/vofa.c
    User/motor/alob.c
    User/motor/anticogging.c
    User/motor/calibration.c
    User/motor/encoder.c
    User/motor/foc_ctrl.c
    User/motor/foc_drv.c
    User/motor/hfsi.c
    User/motor/idpm.c
    User/motor/nlob.c
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

# CubeMX 在未启用 USE_FULL_LL_DRIVER 时仍保留该源文件；此配置下原文件为空，
# 因此只对该第三方文件关闭空翻译单元提示，不降低项目源码的警告等级。
set_source_files_properties(
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c"
    PROPERTIES COMPILE_OPTIONS "-Wno-pedantic"
)

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

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    # 快速周期函数按设计从 Flash 复制到可写 RAM 执行；STM32G4 没有进程级
    # 可执行内存权限边界，因此只关闭 GNU ld 针对该嵌入式布局的通用提示。
    target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        "LINKER:--no-warn-rwx-segments"
    )
endif()

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
