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

# seed_array=(8972 1784 7583 3829 6784 8392 7489 3423 5432 2349 8753 4637 9874 4812 13242 9238 16458 5435 1658 535 4326)
seed_array=(4545 3746 4654 8796 1564 3034 8653 4854 1318 5624) # 8965 6341 4389 6532 3063 2164 1167 7890 6114 2631 9296 1743 8294 4326 4439 7010 3880) #) # 
scan_array=(3) # 8 9)


for j in ${scan_array[@]}
do 
    echo ""
    echo "scan $j"
    echo ""
    ./app/rcs -seed 4545 -scan $j -r 14 -l 100 -phi 360 -timeout 100000 -bias 0.05 -save_pc -save_interp -var_curve
    ./app/rcs_star -seed 4545 -scan $j -r 14 -l 100 -phi 360 -timeout 100000 -bias 0.05 -save_pc -save_interp -var_curve

    for i in ${seed_array[@]}
    do
        echo ""
        echo "seed $i scan $j"
        echo ""
        ./app/rrt -seed $i -scan $j -r 14 -l 100 -phi 360 -timeout 50000 -bias 0.05 -save_pc -save_interp -var_curve
        ./app/aorrt -seed $i -scan $j -r 14 -l 100 -phi 360 -timeout 50000 -bias 0.05 -save_pc -save_interp -var_curve

        # ./app/rrt -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
        # ./app/aorrt -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
        # ./app/rcs -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
        # ./app/rcs_star -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
    done


done
