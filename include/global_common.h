// BSD 3-Clause License

// Copyright (c) 2021, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#pragma once
#ifndef SNP_GLOBAL_COMMON_H
#define SNP_GLOBAL_COMMON_H

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <map>

#include <boost/multi_array.hpp>
#include <Eigen/Dense>

namespace unc::robotics::snp {

#define SE3_T_W 20
#define SE3_R_W 1

#define NORM2_T_W 1
#define NORM2_R_W 10

using Str = std::string;
using Idx = unsigned short;
using SizeType = std::size_t;
using RealNum = double;
using RealUniformDist = std::uniform_real_distribution<RealNum>;
using RealNormalDist = std::normal_distribution<RealNum>;
using IntUniformDist = std::uniform_int_distribution<int>;
constexpr Idx I_INF = std::numeric_limits<Idx>::max();
constexpr RealNum R_INF = std::numeric_limits<RealNum>::infinity();
constexpr RealNum EPS = 1e-10;
constexpr RealNum DEGREE_TO_RAD = M_PI/180.0;
constexpr RealNum RAD_TO_DEGREE = 180.0/M_PI;

using IdxPoint = Eigen::Matrix<Idx, 3, 1, Eigen::ColMajor>;
using IntPoint = Eigen::Matrix<int, 3, 1, Eigen::ColMajor>;
using Vec2 = Eigen::Matrix<RealNum, 2, 1, Eigen::ColMajor>;
using Vec3 = Eigen::Matrix<RealNum, 3, 1, Eigen::ColMajor>;
using Vec4 = Eigen::Matrix<RealNum, 4, 1, Eigen::ColMajor>;
using Mat3 = Eigen::Matrix<RealNum, 3, 3, Eigen::ColMajor>;
using Mat4 = Eigen::Matrix<RealNum, 4, 4, Eigen::ColMajor>;
using Affine = Eigen::Transform<RealNum, 3, Eigen::Affine, Eigen::ColMajor>;
using Quat = Eigen::Quaternion<RealNum>;
using AngleAxis = Eigen::AngleAxis<RealNum>;

using BoolArray2 = boost::multi_array<bool, 2>;
using IdxArray2 = boost::multi_array<Idx, 2>;
using BoolArray3 = boost::multi_array<bool, 3>;
using IdxArray3 = boost::multi_array<Idx, 3>;
using RealArray3 = boost::multi_array<RealNum, 3>;

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

const RealNum kRadCurve = 100.0;
const RealNum kAngleConstraintDegree = 90.0;



// Misc.
Idx global_seed = 1;
bool global_show_logs = true;
bool global_variable_curvature = false;
int scan_number = 9;
Str global_output_file_root = "../data/output/test";
Str global_obstacle_file = "../data/input/remind_00" + std::to_string(scan_number) + "_obstacles.txt";
Str global_skull_file = "../data/input/remind_00" + std::to_string(scan_number) + "_skull_outline_shuffled.txt";
Str global_cost_file = "../data/input/costs.txt";
Str global_healpix_file = "../data/input/HEALPix.txt";
Str needle_parameter_file = "../data/input/needle_parameters.txt";
Str start_and_goal_file = "../data/input/remind_00" + std::to_string(scan_number) + "_start_and_goal_poses.txt";
Str goal_file = "../data/input/remind_00" + std::to_string(scan_number) + "_goal_regions.txt";
Str stats_file = "../data/output/planner_stats.txt";



// Planner behavior control. All parameters use [mm], [rad].
// Use single threads or multiple threads.
bool global_multi_threading = false;
// Position tolerance for the goal state.
RealNum global_goal_pos_tolerance = EPS;
// Orientation tolerance for the goal state.
RealNum goal_ang_tolerance = 0.005;

// Resolution to check if an edge is valid.
RealNum validity_res = 0.5;
RealNum cost_res = 0.1;
// Safe margin for collision detection.
RealNum safe_margin = 2.0;
// Resolution used to reinterpolate the result plan.
RealNum result_res = 2.0;
// The probability of sampling goal state directly.
RealNum global_goal_bias = 0.05;
RealNum global_start_connect_ratio = 0.0;

// Method used to do goal connection.
bool global_dubins = false;
// // When doing spreading, if the start orientation is fixed.
// bool spreading_fix_start_orientation = false;
// // If we allow spreading in all directions, some of the configurations are used
// // for generating new start orientations.
// RealNum start_connect_ratio = 0.05;
// // If the sampler also sample orientation.
// bool sample_orientation = false;
// // For spreading planner.
// RealNum spreading_min_dist = 100.0;

// // For RCS planner.
// RealNum delta_ell_max = 16.0;
// RealNum delta_theta_max = 0.5 * M_PI;
// RealNum delta_ell_min = 0.125;
// RealNum delta_theta_min = 0.157;

// // For RCS* planner.
// unsigned look_ahead = 3;
// RealNum cost_approx_factor = 0.1;

// bool optimal = false;
// bool use_trilinear_interpolation = true;

// Termination control.
// Timeout in milliseconds.
SizeType global_timeout = 1000;
// Maximum number of nodes in the tree.
SizeType global_num_nodes = 10000;
// Number of plans needed for termination.
SizeType global_num_plans_needed = 10;

bool save_ptcloud = false;
bool save_interp = false;





std::tuple<bool, Str, RealNum, RealNum, RealNum> ParseArgs(int argc, char ** argv, RealNum min_curve_rad, RealNum insertion_length, RealNum angle_constraint_degree, Str suffix) {
    
    bool constrain_goal_orientation = false;
    global_multi_threading = false;
    global_variable_curvature = false;
    global_timeout = 5000;

    int i = 1;
    while (i < argc) {
        if (std::strcmp(argv[i], "-r") == 0) {
            min_curve_rad = std::atoi(argv[++i]);
        } 
        else if (std::strcmp(argv[i], "-l") == 0 ) {
            insertion_length = std::atoi(argv[++i]);
        }  
        else if (std::strcmp(argv[i], "-phi") == 0) {
            angle_constraint_degree = std::atoi(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-seed") == 0) {
            global_seed = std::atoi(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-scan") == 0) {
            scan_number = std::atoi(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-suffix") == 0) {
            suffix = suffix + "_" + argv[++i];  
        }
        else if (std::strcmp(argv[i], "-bias") == 0) {
            global_goal_bias = std::stod(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-tau") == 0) {
            global_goal_pos_tolerance = std::stod(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-timeout") == 0) {
            global_timeout = std::atoi(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-nodes") == 0) {
            global_num_nodes = std::atoi(argv[++i]);  
        }
        else if (std::strcmp(argv[i], "-dubins") == 0) {
            global_dubins = true;  
        }
        else if (std::strcmp(argv[i], "-constrain_goal") == 0) {
            constrain_goal_orientation = true;  
        } 
        else if (std::strcmp(argv[i], "-multi") == 0) {
            global_multi_threading = true;  
        } 
        else if (std::strcmp(argv[i], "-save_pc") == 0) {
            save_ptcloud = true;
        }
        else if (std::strcmp(argv[i], "-save_interp") == 0) {
            save_interp = true;
        } 
        else if (std::strcmp(argv[i], "-start_sample") == 0) {
            global_start_connect_ratio = std::stod(argv[++i]);
        }  
        else if (std::strcmp(argv[i], "-stats_file") == 0) {
            stats_file = argv[++i];
        }
        else if (std::strcmp(argv[i], "-var_curve") == 0) {
            global_variable_curvature = true;
        }                                      
        else {
            std::cerr << "Specified arg not supported " << argv[i] << std::endl;
        }
            
        i++;
    }

    Str padded_scan_num = std::to_string(scan_number);
    padded_scan_num = std::string(3 - padded_scan_num.length(), '0') + padded_scan_num;

    start_and_goal_file = "../data/input/remind_" + padded_scan_num + "_start_and_goal_poses.txt";
    global_obstacle_file = "../data/input/remind_" + padded_scan_num + "_obstacles.txt";
    global_skull_file = "../data/input/remind_" + padded_scan_num + "_skull_outline_shuffled.txt";
    goal_file = "../data/input/remind_" + padded_scan_num + "_goal_regions.txt";
    suffix = suffix + "_remind_" + padded_scan_num; 
    
#ifdef HAVE_GLOBAL_VARIABLES
    global::needle_min_curve_rad = min_curve_rad;
    global::angle_constraint_degree = angle_constraint_degree;
#endif

    return {constrain_goal_orientation, suffix, min_curve_rad, insertion_length, angle_constraint_degree};
}




} // namespace unc::robotics::snp

#endif // SNP_GLOBAL_COMMON_H