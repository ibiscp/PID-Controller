#! /bin/bash
sudo rm -rf build
mkdir build && cd build
cmake .. && make
./pid
