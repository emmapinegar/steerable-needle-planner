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

# https://www.digitalocean.com/community/tutorials/arrays-in-shell-scripts

seed_array=(16458 5435 1658 535 4326) # 8972 1784 7583 3829 6784 8392 7489 3423 5432 2349 8753 4637 9874 4812 13242 9238) 
# for i in ${seed_array[@]}
# do 
#     ./app/rrt 0 1 $i 1 25
# done

for i in ${seed_array[@]}
do 
    # ./app/rrt 0 1 $i 1 25
    # ./app/aorrt 0 1 $i 1 25
    ./app/rcs 0 1 $i 1 25
    ./app/rcs_star 0 1 $i 1 25
done
# 
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

# ./app/rcs 0 1 45843 1 25
# ./app/rcs 0 1 1658 1 25
# ./app/rcs 0 1 535 1 25
# ./app/rcs 0 1 45843 8 25
# ./app/rcs 0 1 1658 8 25
# ./app/rcs 0 1 535 8 25
# ./app/rcs 0 1 45843 9 25
# ./app/rcs 0 1 1658 9 25
# ./app/rcs 0 1 535 9 25


# ./app/rcs_star 0 1 45843 1 25
# ./app/rcs_star 0 1 1658 1 25
# ./app/rcs_star 0 1 535 1 25
# ./app/rcs_star 0 1 45843 8 25
# ./app/rcs_star 0 1 1658 8 25
# ./app/rcs_star 0 1 535 8 25
# ./app/rcs_star 0 1 45843 9 25
# ./app/rcs_star 0 1 1658 9 25
# ./app/rcs_star 0 1 535 9 25