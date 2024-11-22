#!/bin/bash

# install gcc/g++

# printf 'Operating System: %s \n\n' "$OSTYPE"
# if [[ "$OSTYPE" == "linux-gnu"* ]]; then
#     # install dependencies 
#     sudo apt install cmake libeigen3-dev libboost-all-dev pip;
#     # get the right version of numpy
#     pip3 install open3d
#     pip3 install numpy==1.26.4
#     # install dependencies for mpt
#     sudo apt install ninja eigen3 libfcl libassimp;

# elif [[ "$OSTYPE" == "darwin"* ]]; then
#     # Mac OSX
#     printf 'Installing Mac version of dependencies... \n' 
#     brew install cmake eigen boost
#     brew install ninja assimp libomp
# else
#     printf 'Operating system is not compatible with this install script! Exiting....'
#     exit 1
# fi    



# build everything to make sure we're up to date
# cd ./build
# cmake ..
# make


# run some experiments
cd ./build

# ./app/rrt 0 1 475843 1 25
# ./app/rrt 0 1 16458 1 25
# ./app/rrt 0 1 5435 1 25
# ./app/rrt 0 1 475843 8 25
# ./app/rrt 0 1 16458 8 25
# ./app/rrt 0 1 5435 8 25
# ./app/rrt 0 1 475843 9 25
# ./app/rrt 0 1 16458 9 25
# ./app/rrt 0 1 5435 9 25

# ./app/aorrt 0 1 475843 1 25
# ./app/aorrt 0 1 16458 1 25
# ./app/aorrt 0 1 5435 1 25
# ./app/aorrt 0 1 475843 8 25
# ./app/aorrt 0 1 16458 8 25
# ./app/aorrt 0 1 5435 8 25
# ./app/aorrt 0 1 475843 9 25
# ./app/aorrt 0 1 16458 9 25
# ./app/aorrt 0 1 5435 9 25


# ./app/rcs 0 1 475843 1 25
# ./app/rcs 0 1 16458 1 25
# ./app/rcs 0 1 5435 1 25
# ./app/rcs 0 1 475843 8 25
# ./app/rcs 0 1 16458 8 25
# ./app/rcs 0 1 5435 8 25
# ./app/rcs 0 1 475843 9 25
# ./app/rcs 0 1 16458 9 25
# ./app/rcs 0 1 5435 9 25


# ./app/rcs_star 0 1 475843 1 25
# ./app/rcs_star 0 1 16458 1 25
# ./app/rcs_star 0 1 5435 1 25
# ./app/rcs_star 0 1 475843 8 25
# ./app/rcs_star 0 1 16458 8 25
# ./app/rcs_star 0 1 5435 8 25
# ./app/rcs_star 0 1 475843 9 25
# ./app/rcs_star 0 1 16458 9 25
# ./app/rcs_star 0 1 5435 9 25

# ./app/rcs 0 1 475843 1 25
# ./app/rcs 0 1 475843 8 25
# ./app/rcs 0 1 475843 9 25

# ./app/rcs_star 0 1 475843 1 25
# ./app/rcs_star 0 1 475843 8 25
# ./app/rcs_star 0 1 475843 9 25




# ./app/rrt_spreading 1 475843 1 25
# ./app/rrt_spreading 1 16458 1 25
# ./app/rrt_spreading 1 5435 1 25
# ./app/rrt_spreading 1 475843 8 25
# ./app/rrt_spreading 1 16458 8 25
# ./app/rrt_spreading 1 5435 8 25
# ./app/rrt_spreading 1 475843 9 25
# ./app/rrt_spreading 1 16458 9 25
# ./app/rrt_spreading 1 5435 9 25

# ./app/aorrt_spreading 1 475843 1 25
# ./app/aorrt_spreading 1 16458 1 25
# ./app/aorrt_spreading 1 5435 1 25
# ./app/aorrt_spreading 1 475843 8 25
# ./app/aorrt_spreading 1 16458 8 25
# ./app/aorrt_spreading 1 5435 8 25
# ./app/aorrt_spreading 1 475843 9 25
# ./app/aorrt_spreading 1 16458 9 25
# ./app/aorrt_spreading 1 5435 9 25

# ./app/rcs_spreading 1 475843 1 25
# ./app/rcs_spreading 1 16458 1 25
# ./app/rcs_spreading 1 5435 1 25
# ./app/rcs_spreading 1 475843 8 25
# ./app/rcs_spreading 1 16458 8 25
# ./app/rcs_spreading 1 5435 8 25
# ./app/rcs_spreading 1 475843 9 25
# ./app/rcs_spreading 1 16458 9 25
# ./app/rcs_spreading 1 5435 9 25

./app/rcs 0 1 45843 1 25
./app/rcs 0 1 1658 1 25
./app/rcs 0 1 535 1 25
./app/rcs 0 1 45843 8 25
./app/rcs 0 1 1658 8 25
./app/rcs 0 1 535 8 25
./app/rcs 0 1 45843 9 25
./app/rcs 0 1 1658 9 25
./app/rcs 0 1 535 9 25


./app/rcs_star 0 1 45843 1 25
./app/rcs_star 0 1 1658 1 25
./app/rcs_star 0 1 535 1 25
./app/rcs_star 0 1 45843 8 25
./app/rcs_star 0 1 1658 8 25
./app/rcs_star 0 1 535 8 25
./app/rcs_star 0 1 45843 9 25
./app/rcs_star 0 1 1658 9 25
./app/rcs_star 0 1 535 9 25