#!/bin/bash

# run some experiments
cd ./build

# https://www.digitalocean.com/community/tutorials/arrays-in-shell-scripts

# seed_array=(8972 1784 7583 3829 6784 8392 7489 8753 4637 9874 4812 13242 9238 16458 5435 1658 535 4326)
seed_array=(8965) # 4389 6532 3063 2164 1167 7890 6114 2631) # 9296 1743 8294 4326 4439 7010 3880 3423 5432 2349) #(4545 3746 4654 8796 1564 3034 8653 4854 1318 5624)
scan_array=(1) # 3 6 8 9 10 13) # 15) # 8 9)
start_array=(0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25)


for j in ${scan_array[@]}
do
    for k in ${start_array[@]}
    do
        echo ""
        echo "scan $j"
        echo ""
        # ./app/rcs -seed 4545 -scan $j -r 14 -l 100 -phi 360 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k
        # ./app/rcs_star -seed 4545 -scan $j -r 14 -l 100 -phi 360 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k

        for i in ${seed_array[@]}
        do
            echo ""
            echo "seed $i scan $j"
            echo ""
            ./app/rrt -seed $i -scan $j -r 14 -l 100 -phi 360 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k
            # ./app/aorrt -seed $i -scan $j -r 14 -l 100 -phi 360 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k

            # ./app/rrt -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
            # ./app/aorrt -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
            # ./app/rcs -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
            # ./app/rcs_star -seed $i -scan $j -r 20 -l 200 -phi 360 -timeout 240000 -bias 0.05 -multi -stats_file "../data/output/planner_multi_stats.txt"
        done
    done

done
