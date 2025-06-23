mkdir -p "${CI_PROJECT_DIR}/build_emu"
cd "${CI_PROJECT_DIR}/build_emu"
cmake .. -G Ninja -DEMU=ON 
ninja