# Copyright (c) 2022-2024, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
cmake_minimum_required(VERSION ${TI_MIN_CMAKE_VERSION})

# This is a top-level toolchain file for the gcc compiler, defining the most general options and variables as well
# as any special commands or invocations needed by the compiler.
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(TI_TOOLCHAIN_NAME gcc)
set(TI_TOOLCHAIN_LINKER_FILE_EXTENSION lds)

#Set compilers and archiver
if(WIN32)
    set(CMAKE_C_COMPILER "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc.exe")
    set(CMAKE_CXX_COMPILER "${GCC_ARMCOMPILER}/bin/arm-none-eabi-g++.exe")
    set(CMAKE_ASM_COMPILER "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc.exe")
    set(CMAKE_AR "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc-ar.exe")
    set(CMAKE_RANLIB "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc-ranlib.exe")
else()
    set(CMAKE_C_COMPILER "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc")
    set(CMAKE_CXX_COMPILER "${GCC_ARMCOMPILER}/bin/arm-none-eabi-g++")
    set(CMAKE_ASM_COMPILER "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc")
    set(CMAKE_AR "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc-ar")
    set(CMAKE_RANLIB "${GCC_ARMCOMPILER}/bin/arm-none-eabi-gcc-ranlib")
endif()

#Specify how the compilers should be invoked
set(CMAKE_C_COMPILE_OBJECT
    "<CMAKE_C_COMPILER> -c -MD -MF $$@.dep <DEFINES> <INCLUDES> <FLAGS> -o <OBJECT> -MD -MF <OBJECT>.d -c <SOURCE>"
)
set(CMAKE_CXX_COMPILE_OBJECT
    "<CMAKE_CXX_COMPILER> -c -MD -MF $$@.dep <DEFINES> <INCLUDES> <FLAGS> -o <OBJECT> -MD -MF <OBJECT>.d -c <SOURCE>"
)
set(CMAKE_ASM_COMPILE_OBJECT
    "<CMAKE_ASM_COMPILER> -c -x assembler-with-cpp <DEFINES> <INCLUDES> <FLAGS> -o <OBJECT> -c <SOURCE>"
)
set(CMAKE_AR_FLAGS -r)

# Needed otherwise the CMake compiler checks will complain and abort
set(CMAKE_C_COMPILER_ID_RUN TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_WORKS TRUE)

set(CMAKE_CXX_COMPILER_ID_RUN TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)

if(NOT TARGET TOOLCHAIN_gcc)
    add_library(TOOLCHAIN_gcc INTERFACE IMPORTED)
    target_compile_options(
        TOOLCHAIN_gcc
        INTERFACE
            $<$<STREQUAL:$<TARGET_PROPERTY:TI_CFLAGS_OVERRIDE>,>:
            # This part included if TI_CFLAGS_OVERRIDE not defined
            # Use GNU extensions to C and C++ to enable functions like strnlen.
            $<$<COMPILE_LANGUAGE:C>:-std=gnu17>
            $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++23>
            -mthumb
            $<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:-g
            -mabi=aapcs
            -Dgcc
            -ffunction-sections
            -fdata-sections>
            $<$<AND:$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>,$<CONFIG:Release>>:-O3>
            $<$<AND:$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>,$<CONFIG:Debug>>:-O0>
            $<$<AND:$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>,$<BOOL:${TI_CMAKE_COMMON_ENABLE_ALL_WARNINGS}>>:-Wall>
            $<$<AND:$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>,$<BOOL:${TI_CMAKE_COMMON_WARNINGS_AS_ERRORS}>>:-Werror>
            >
            # If TI_CFLAGS_OVERRIDE, use it exclusively
            $<$<NOT:$<STREQUAL:$<TARGET_PROPERTY:TI_CFLAGS_OVERRIDE>,>>:$<TARGET_PROPERTY:TI_CFLAGS_OVERRIDE>>
    )
    target_link_options(
        TOOLCHAIN_gcc
        INTERFACE
        $<$<STREQUAL:$<TARGET_PROPERTY:TI_LFLAGS_OVERRIDE>,>:
        # This part included if TI_LFLAGS_OVERRIDE not defined
        # If linker-file property exists, add linker file
        $<$<NOT:$<STREQUAL:$<TARGET_PROPERTY:TI_LINKER_COMMAND_FILE>,>>:-Wl,-T,$<TARGET_PROPERTY:TI_LINKER_COMMAND_FILE>>
        # If map-file property exists, set map file
        $<$<NOT:$<STREQUAL:$<TARGET_PROPERTY:TI_LINKER_MAP_FILE>,>>:-Wl,-Map,$<TARGET_PROPERTY:TI_LINKER_MAP_FILE>>
        # -specs=nano.specs  # Removed: nano printf lacks %lld support, crashing utest timing output.
        # Disables 0x10000 sector allocation boundaries, which interfere
        # with the SPE layouts and prevent proper secure operation
        -Wl,--nmagic
        $<$<BOOL:${TI_CMAKE_COMMON_WARNINGS_AS_ERRORS}>:-Wl,--fatal-warnings>
        # End of section for TI_LFLAGS_OVERRIDE not defined
        >
        # If TI_CFLAGS_OVERRIDE, use it exclusively
        $<$<NOT:$<STREQUAL:$<TARGET_PROPERTY:TI_CFLAGS_OVERRIDE>,>>:$<TARGET_PROPERTY:TI_LFLAGS_OVERRIDE>>
    )

    add_library(TOOLCHAIN_gcc_m0p INTERFACE IMPORTED)
    target_link_libraries(TOOLCHAIN_gcc_m0p INTERFACE TOOLCHAIN_gcc)
    target_compile_options(TOOLCHAIN_gcc_m0p INTERFACE -mcpu=cortex-m0plus -mfloat-abi=soft)
    add_library(CMakeCommon::gcc_m0p ALIAS TOOLCHAIN_gcc_m0p)

    add_library(TOOLCHAIN_gcc_m3 INTERFACE IMPORTED)
    target_link_libraries(TOOLCHAIN_gcc_m3 INTERFACE TOOLCHAIN_gcc)
    target_compile_options(TOOLCHAIN_gcc_m3 INTERFACE -mcpu=cortex-m3 -mfloat-abi=soft)
    add_library(CMakeCommon::gcc_m3 ALIAS TOOLCHAIN_gcc_m3)

    add_library(TOOLCHAIN_gcc_m4 INTERFACE IMPORTED)
    target_link_libraries(TOOLCHAIN_gcc_m4 INTERFACE TOOLCHAIN_gcc)
    target_compile_options(TOOLCHAIN_gcc_m4 INTERFACE -mcpu=cortex-m4 -mfloat-abi=soft)
    add_library(CMakeCommon::gcc_m4 ALIAS TOOLCHAIN_gcc_m4)

    add_library(TOOLCHAIN_gcc_m4f INTERFACE IMPORTED)
    target_link_libraries(TOOLCHAIN_gcc_m4f INTERFACE TOOLCHAIN_gcc)
    target_compile_options(TOOLCHAIN_gcc_m4f INTERFACE -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16)
    add_library(CMakeCommon::gcc_m4f ALIAS TOOLCHAIN_gcc_m4f)

    add_library(TOOLCHAIN_gcc_m33f INTERFACE IMPORTED)
    target_link_libraries(TOOLCHAIN_gcc_m33f INTERFACE TOOLCHAIN_gcc)
    target_compile_options(TOOLCHAIN_gcc_m33f INTERFACE -mcpu=cortex-m33 -mfloat-abi=hard -mfpu=fpv5-sp-d16)
    add_library(CMakeCommon::gcc_m33f ALIAS TOOLCHAIN_gcc_m33f)
endif()
