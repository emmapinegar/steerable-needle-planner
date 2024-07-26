#!/bin/bash

# install gcc/g++

printf 'Operating System: %s \n\n' "$OSTYPE"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # install dependencies 
    sudo apt install cmake libeigen3-dev libboost-all-dev pip;
    # get the right version of numpy
    pip3 install open3d
    pip3 install numpy==1.26.4
    # install dependencies for mpt
    sudo apt install ninja eigen3 libfcl libassimp;

elif [[ "$OSTYPE" == "darwin"* ]]; then
    # Mac OSX
    printf 'Installing Mac version of dependencies... \n' 
    brew install cmake eigen boost
    brew install ninja assimp libomp
else
    printf 'Operating system is not compatible with this install script! Exiting....'
    exit 1
fi    


# initialize submodules
git submodule update --init --recursive


# create necessary folders
mkdir -p ./build
mkdir -p ./data/output



# build everything
cd ./build
cmake ..
make


