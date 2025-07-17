mkdir -p "${CI_PROJECT_DIR}/build_stm32"
cd "${CI_PROJECT_DIR}/build_stm32"
cmake .. -G Ninja -DSTM32=ON -DCMAKE_TOOLCHAIN_FILE=stm32/gcc-arm-none-eabi.cmake
ninja