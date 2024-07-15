#!/bin/bash

# install gcc/g++


# install dependencies 
sudo apt install cmake libeigen3-dev libboost-all-dev pip
pip3 install open3d

# install dependencies for mpt
sudo apt install ninja eigen3 libfcl libassimp


# get the right version of numpy
pip install numpy==1.26.4


# initialize submodules
git submodule update --init --recursive


# create necessary folders
mkdir -p ./build
mkdir -p ./data/output



# build everything
cd ./build
cmake ..
make


