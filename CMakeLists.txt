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

# C Plus Plus Extention
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SparkMoveCar)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Micro-XRCE-DDS-Client config for STM32 + RT-Thread + custom UART transport
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

set(UCDR_SUPERBUILD OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(UCDR_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(UCDR_PIC OFF CACHE BOOL "" FORCE)
set(UCDR_ISOLATED_INSTALL OFF CACHE BOOL "" FORCE)

set(UCLIENT_SUPERBUILD OFF CACHE BOOL "" FORCE)
set(UCLIENT_PIC OFF CACHE BOOL "" FORCE)
set(UCLIENT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(UCLIENT_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(UCLIENT_INSTALL_EXAMPLES OFF CACHE BOOL "" FORCE)

set(UCLIENT_PROFILE_DISCOVERY OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_UDP OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_TCP OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_SERIAL OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_CAN OFF CACHE BOOL "" FORCE)

set(UCLIENT_PROFILE_CUSTOM_TRANSPORT ON CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_STREAM_FRAMING ON CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_MULTITHREAD OFF CACHE BOOL "" FORCE)
set(UCLIENT_PROFILE_SHARED_MEMORY OFF CACHE BOOL "" FORCE)
set(UCLIENT_CUSTOM_TRANSPORT_MTU 256 CACHE STRING "" FORCE)

add_subdirectory(cmake/stm32cubemx)

add_subdirectory(
    Middlewares/Third_Party/Micro-CDR
    ${CMAKE_BINARY_DIR}/microcdr
)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(
    Middlewares/Third_Party/Micro-XRCE-DDS-Client
    ${CMAKE_BINARY_DIR}/microxrcedds_client
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    microxrcedds_client
)


# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)
# No warnings 
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE 
    -Werror
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/app_main.cpp
    Modules/FIFO/fifo.c
    Bsp/drv_bsp.c
    Bsp/drv_can.c
    Bsp/drv_uart.c
    App/app_test.c
    Device/motor_dji.c
    Device/serial_plotter.c
    Modules/PID/pidcontroller.c
    Device/drv_dm4310.c
    Device/dev_dm4310.c
    Bsp/drv_math.c
    Bsp/drv_imu.c
    Modules/QuaternionEKF/QuaternionEKF.c
    Modules/QuaternionEKF/kalman_filter.c
    Device/dev_dm_imu.c
    Device/N630/dev_n630.c
    Bsp/bsp_sbus.c
    App/Chassis/chassis.c
    App/Micro_Xrce_DDS/uxrce_app.c
    App/Micro_Xrce_DDS/uxrce_transport_uart.c
    App/Micro_Xrce_DDS/uxrce_time_port.c

    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Modules/FIFO
    Modules/PID
    Bsp
    Device
    App
    Modules/QuaternionEKF
    Drivers/CMSIS/DSP/Include
    Device/N630
    App/Chassis
    App/Micro_Xrce_DDS
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

# 强制指定特定的 .c 文件使用 C++ 编译器 (CXX) 进行编译
set_source_files_properties(
    Bsp/drv_bsp.c 
    Bsp/drv_can.c 
    PROPERTIES LANGUAGE CXX
)
