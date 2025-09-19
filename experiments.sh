#!/bin/bash

# run some experiments
cd ./build

# seed_array=(8972 1784 7583 3829 6784 8392 7489 8753 4637 9874 4812 13242 9238 16458 5435 1658 535 4326 4389 6532 3063 2164 1167 7890 6114 2631 9296 1743 8294 4326 4439 7010 3880 3423 5432 2349) #(4545 3746 4654 8796 1564 3034 8653 4854 1318 5624)
seed_array=(8965)
scan_array=(1) # https://www.digitalocean.com/community/tutorials/arrays-in-shell-scripts




for j in ${scan_array[@]}
do

    for i in ${seed_array[@]}
    do
        echo ""
        echo "seed $i scan $j start $k"
        echo ""
        # ./app/rrt -seed $i -scan $j -r 20 -l 100 -phi 180 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k -stats_file "./../data/output/planner_stats.txt"
        # ./app/aorrt -seed $i -scan $j -r 20 -l 100 -phi 180 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k -stats_file "./../data/output/planner_stats.txt"
        # ./app/rcs -seed 8965 -scan $j -r 20 -l 100 -phi 180 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k -stats_file "./../data/output/planner_stats.txt"
        # ./app/rcs_star -seed 8965 -scan $j -r 20 -l 100 -phi 180 -timeout 10000 -bias 0.05 -save_pc -save_interp -var_curve -sg_index $k -stats_file "./../data/output/planner_stats.txt"

        # ./app/rrt -seed $i -scan $j -r 25 -l 100 -phi 180 -timeout 100000 -bias 0.05 -var_curve -sg_index 0 -stats_file "./../data/output/planner_stats.txt" -num_sg 100 -save_pc
        # ./app/aorrt -seed $i -scan $j -r 25 -l 100 -phi 180 -timeout 100000 -bias 0.05 -var_curve -sg_index 0 -stats_file "./../data/output/planner_stats.txt" -num_sg 100 -save_pc





        ./app/rrt -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve
        ./app/aorrt -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve
        ./app/rcs -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve
        ./app/rcs_star -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve



        ./app/rrt -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve
        ./app/aorrt -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve
        ./app/rcs -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve
        ./app/rcs_star -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" -var_curve


        ./app/rrt -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 
        ./app/aorrt -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 
        ./app/rcs -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 
        ./app/rcs_star -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 



        ./app/rrt -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 
        ./app/aorrt -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 
        ./app/rcs -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 
        ./app/rcs_star -seed 8965 -scan $j -r 15 -l 100 -phi 180 -timeout 10000 -bias 0.05 -sg_index 0 -num_sg 10 -stats_file "./../data/output/planner_stats.txt" 


        
    done

done
