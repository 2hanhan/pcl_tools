#!/bin/bash -e
echo "Configuring and building ..."
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
#cmake .. -DCMAKE_BUILD_TYPE=Debug

make -j8
