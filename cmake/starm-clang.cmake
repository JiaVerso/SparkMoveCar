set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID Clang)
set(CMAKE_CXX_COMPILER_ID Clang)

# Some default llvm settings
set(TOOLCHAIN_PREFIX                starm-)

## Linux系统 --- 取消这部分注释即可
# Ubuntu22.04 
# set(ST_COMPILER_FOR_LINUX           /opt/st/stm32cubeclt_1.21.0/st-arm-clang/bin)
## Ubuntu22.04 
##set(CMAKE_C_COMPILER                ${ST_COMPILER_FOR_LINUX}/starm-clang)
##set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
##set(CMAKE_CXX_COMPILER              ${ST_COMPILER_FOR_LINUX}/starm-clang++)
# # 如果下面还有这些工具，也建议一并改为绝对路径
# set(CMAKE_LINKER                    ${ST_COMPILER_FOR_LINUX}/starm-clang)
# set(CMAKE_OBJCOPY                   ${ST_COMPILER_FOR_LINUX}/starm-objcopy)
# set(CMAKE_SIZE                      ${ST_COMPILER_FOR_LINUX}/starm-size)

# Win10 - C:\ST\STM32CubeCLT\STMicroelectronics_LLVM_ARM\bin -
set(ST_COMPILER_FOR_WIN "C:/ST/STM32CubeCLT_1.21.0/st-arm-clang/bin")
set(CMAKE_C_COMPILER   "${ST_COMPILER_FOR_WIN}/starm-clang.exe" CACHE PATH "" FORCE)
set(CMAKE_CXX_COMPILER "${ST_COMPILER_FOR_WIN}/starm-clang++.exe" CACHE PATH "" FORCE)
set(CMAKE_ASM_COMPILER "${CMAKE_C_COMPILER}" CACHE PATH "" FORCE)
# 如果下面还有这些工具，也建议一并改为绝对路径
set(CMAKE_LINKER       "${ST_COMPILER_FOR_WIN}/starm-clang.exe")
set(CMAKE_OBJCOPY      "${ST_COMPILER_FOR_WIN}/starm-objcopy.exe" CACHE PATH "" FORCE)
set(CMAKE_SIZE         "${ST_COMPILER_FOR_WIN}/starm-size.exe" CACHE PATH "" FORCE)


set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# STARM_TOOLCHAIN_CONFIG allows you to choose the toolchain configuration.
# Possible values are:
#  "STARM_HYBRID"   : Hybrid configuration using starm-clang Assemler and Compiler and GNU Linker
#  "STARM_NEWLIB"   : starm-clang toolchain with NEWLIB C library
#  "STARM_PICOLIBC" : starm-clang toolchain with PICOLIBC C library
set(STARM_TOOLCHAIN_CONFIG "STARM_PICOLIBC")

if(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_HYBRID")
  set(TOOLCHAIN_MULTILIBS "--multi-lib-config=\"$ENV{CLANG_GCC_CMSIS_COMPILER}/multilib.gnu_tools_for_stm32.yaml\" --gcc-toolchain=\"$ENV{GCC_TOOLCHAIN_ROOT}/..\"")
elseif (STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_NEWLIB")
  set(TOOLCHAIN_MULTILIBS "--config=newlib.cfg")
endif()

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ${TOOLCHAIN_MULTILIBS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -ftls-model=local-exec -fstack-usage")

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Oz -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Oz -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")

if (STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_HYBRID")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --gcc-specs=nano.specs")
  set(TOOLCHAIN_LINK_LIBRARIES "m")
elseif(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_NEWLIB")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lcrt0-nosys")
elseif(STARM_TOOLCHAIN_CONFIG STREQUAL "STARM_PICOLIBC")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lcrt0-hosted -z norelro")

endif()

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F427XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -z noexecstack")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage ")
