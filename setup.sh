#make a build dir cd into and then run cmake .. -G Ninja

mkdir build
cp codegen.sh ./build
cd build
cmake .. -G Ninja -DEMU=ON
