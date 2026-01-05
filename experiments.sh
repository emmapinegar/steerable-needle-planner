#!/bin/bash

# run some experiments
cd ./build

# seed_array=(8972 1784 7583 3829 6784 8392 7489 8753 4637 9874 4812 13242 9238 16458 5435 1658 535 4326 4389 6532 3063 2164 1167 7890 6114 2631 9296 1743 8294 4326 4439 7010 3880 3423 5432 2349) #(4545 3746 4654 8796 1564 3034 8653 4854 1318 5624)
seed_array=(8965)
scan_array=(1)                  # https://www.digitalocean.com/community/tutorials/arrays-in-shell-scripts
phi_arr=(180)
rads=(25)
sgs=(1500)
seed=8965
scan=1
ell=100
timeout=100000
multi_timeout=10000
bias=0.05
num_sg=1
stats_file="./../data/output/planner_stats.txt"
multi_stats_file="./../data/output/planner_stats_multi_test.txt"
test_file="./../data/output/planner_stats_multi_debug.txt"
i=0

# for j in ${scan_array[@]}
# do

#     for i in {1500..2000..500}
#     do
#         echo ""
#         echo "seed $i scan $j start $k"
#         echo ""
 
#         # ./app/rrt -seed 8965 -scan $j -r 250 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index $i -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 100 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 1500 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 50 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 1000 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 25 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 500 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 0 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
        
#     done
# done



# for j in ${scan_array[@]}
# do

#     for i in ${seed_array[@]}
#     do
#         echo ""
#         echo "seed $i scan $j start $k"
#         echo ""
 
#         ./app/rrt -seed 8965 -scan $j -r 250 -l 100 -phi 90 -timeout 100000 -bias 0.05 -sg_index 2341 -num_sg 1 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 100 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 1500 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 50 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 1000 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 25 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 500 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
#         # ./app/rrt -seed 8965 -scan $j -r 15 -l 100 -phi 90 -timeout 10 -bias 0.05 -sg_index 0 -num_sg 500 -stats_file "./../data/output/planner_stats.txt"
        
#     done
# done


for j in ${scan_array[@]}
do

    for ((i=0;i<${#rads[@]};i++));  
    do
        echo ""
        echo "seed $i scan $j radius ${rads[$i]}"
        echo ""
 
        # https://www.geeksforgeeks.org/linux-unix/array-basics-shell-scripting-set-2-using-loops/


            ./app/rrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -var_curve -multi
            ./app/aorrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -var_curve -multi
            ./app/rcs -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -var_curve -multi
            ./app/rcs_star -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -var_curve -multi

            # ./app/rrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -multi
            # ./app/aorrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -multi
            # ./app/rcs -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -multi
            # ./app/rcs_star -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi 180 -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $test_file -multi


    done  

done



# for j in ${scan_array[@]}
# do

#     for ((i=0;i<${#rads[@]};i++));  
#     do
#         echo ""
#         echo "seed $i scan $j radius ${rads[$i]}"
#         echo ""
 
#         # https://www.geeksforgeeks.org/linux-unix/array-basics-shell-scripting-set-2-using-loops/

#         for p in ${phi_arr[@]}
#         do

#             ./app/rrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file -var_curve
#             # ./app/aorrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file -var_curve
#             # ./app/rcs -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file -var_curve
#             # ./app/rcs_star -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file -var_curve

#             # ./app/rrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file
#             # ./app/aorrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file
#             # ./app/rcs -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file
#             # ./app/rcs_star -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $stats_file

#         done
        
#     done  

# done



# for j in ${scan_array[@]}
# do

#     for p in ${phi_arr[@]}
#     do

#         for ((i=0;i<${#rads[@]};i++));                                        # https://stackoverflow.com/questions/169511/how-do-i-iterate-over-a-range-of-numbers-defined-by-variables-in-bash
#         do
#             echo ""
#             echo "seed $i scan $j radius ${rads[$i]}"
#             echo ""
 
#             ./app/rrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -var_curve -multi
#             ./app/aorrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -var_curve -multi
#             ./app/rcs -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -var_curve -multi
#             ./app/rcs_star -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -var_curve -multi
#         done


#         # for ((i=0;i<${#rads[@]};i++));                                        # https://stackoverflow.com/questions/169511/how-do-i-iterate-over-a-range-of-numbers-defined-by-variables-in-bash
#         # do
#         #     echo ""
#         #     echo "seed $i scan $j radius ${rads[$i]}"
#         #     echo ""

#         #     ./app/rrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -multi
#         #     ./app/aorrt -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -multi
#         #     ./app/rcs -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -multi
#         #     ./app/rcs_star -seed $seed -scan $j -r ${rads[$i]} -l $ell -phi $p -timeout $multi_timeout -bias $bias -sg_index ${sgs[$i]} -num_sg $num_sg -stats_file $multi_stats_file -multi
#         # done
        
#     done

# done
