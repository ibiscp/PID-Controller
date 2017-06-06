#! /bin/bash
sudo rm -rf build
mkdir build && cd build
clear
cmake .. && make
./pid
