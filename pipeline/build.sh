#!/bin/bash
# Look for the emu folder
EMU_DIR=$(find . -type d -name "emu" | head -n 1)
# Look for stm32 folder
STM32_DIR=$(find . -type d -name "stm32" | head -n 1)
# If the emu folder is found, build
if [ -d "$EMU_DIR" ]; then
    echo "Building emu..."
    rm -rf "build_emu"
    mkdir -p "build_emu"
    cd "build_emu"
    cmake .. -G Ninja -DEMU=ON
    ninja
    cd ..
else
    echo "No emu folder found."
fi
# If the stm32 folder is found, build
if [ -d "$STM32_DIR" ]; then
    echo "Building stm32..."
    rm -rf "build_stm"
    mkdir -p "build_stm"
    cd "build_stm"
    cmake .. -G Ninja -DSTM32=ON -DCMAKE_TOOLCHAIN_FILE=stm32/gcc-arm-none-eabi.cmake
    ninja
    cd ..
# If nether folder is found, print a message
else
    echo "No stm folder found."
fi