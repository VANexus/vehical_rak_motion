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
set(CMAKE_PROJECT_NAME vehical_rak_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Core/Src/app_car.c
    Core/Src/app_wheel_control.c
    Core/Src/app_can.c
    Core/Src/app_can_control_policy.c
    Core/Src/app_serial.c
    Core/Src/app_serial_session.c
    Core/Src/app_protocol_v2.c
    Core/Src/app_imu.c
    Core/Src/app_oled.c
    Core/Src/app_servo.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

#
# ---------------------------------------------------------------------------
# 以下为手工添加。本文件只生成一次，CubeMX 重新生成不会覆盖；
# cmake/stm32cubemx/CMakeLists.txt 会被覆盖，自定义内容一律写在这里。
# ---------------------------------------------------------------------------
#
# [AI]  额外告警。-Wall 已在 cmake/gcc-arm-none-eabi.cmake 里，但那份是
# CubeMX 生成的、重生成会覆盖，所以补充项写在这里。只作用于本 target 的
# 源文件（app_*.c 与 MX_Application_Src），不波及 HAL/FreeRTOS 库。
#   -Wdouble-promotion 对本仓尤其重要：app_car/app_imu/app_oled 大量 float
#   运算，漏写一个 'f' 后缀就会静默插入一次双精度提升（软浮点，慢且吃栈）。
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wextra
    -Wshadow
    -Wdouble-promotion
    -Wundef
)
#
# 链接完成后生成烧录/存档产物 .hex 与 .bin（STM32CubeProgrammer 两者都收），
# 并打印一次体积占用，便于两端对比。
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex / .bin"
)
