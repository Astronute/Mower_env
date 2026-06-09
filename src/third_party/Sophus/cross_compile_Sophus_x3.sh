#!/usr/bin/env bash

export LD_LIBRARY_PATH=/home/clink/Gitee/x3_cross/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

if [ ! -d "build" ]; then
    mkdir build 
fi

cd build

cmake .. -DCMAKE_C_COMPILER=/home/clink/Gitee/x3_cross/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=/home/clink/Gitee/x3_cross/gcc-ubuntu-9.3.0-2020.03-x86_64-aarch64-linux-gnu/bin/aarch64-linux-gnu-g++
make

