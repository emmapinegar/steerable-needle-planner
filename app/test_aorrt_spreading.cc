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

#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>

#include "global_parameters.h"
#include "test_utils.h"
#include "problem_config.h"
#include "needle_scenario.h"
#include "needle_prrt.h"

using namespace unc::robotics::snp;

int main(int argc, char** argv) {
    Str const date_and_time = utils::DateAndTime();

    // needle parameter file is defined in global_common.h
    auto [min_curve_rad, needle_diameter, insertion_length, angle_constraint_degree]
        = utils::ReadNeedleParameters(needle_parameter_file, false);

    bool constrain_goal_orientation = false;
    global_multi_threading = false;
    Str suffix = "_aorrt_spreading";
    global_timeout = 5000;
    double start_sample = 0.0;
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
        else if (std::strcmp(argv[i], "-mode") == 0) {
            Mode = std::atoi(argv[++i]);  
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
            start_sample = std::stod(argv[++i]);
        }  
        else if (std::strcmp(argv[i], "-stats_file") == 0) {
            stats_file = argv[++i];
        }                                   
        else {
            std::cerr << "Specified arg not supported " << argv[i] << std::endl;
        }
            
        i++;
    }

#ifdef HAVE_GLOBAL_VARIABLES
    global::needle_min_curve_rad = min_curve_rad;
    global::angle_constraint_degree = angle_constraint_degree;
#endif

    start_and_goal_file = "../data/input/remind_00" + std::to_string(scan_number) + "_start_and_goal_poses.txt";
    global_obstacle_file = "../data/input/remind_obstacles_00" + std::to_string(scan_number) + "_outline_shuffled.txt";
    goal_file = "../data/input/remind_00" + std::to_string(scan_number) + "_goal_regions.txt";
    suffix = suffix + "_remind_00" + std::to_string(scan_number);

    ConfigPtr cfg(new ProblemConfig(constrain_goal_orientation,
                                    min_curve_rad,
                                    needle_diameter,
                                    insertion_length,
                                    angle_constraint_degree));


    // start_and_goal_file is defined in global_common.h 
    auto [start_p, start_q] = utils::ReadStart(start_and_goal_file);

    cfg->output_file_root = "../data/output/" + date_and_time + suffix;
    cfg->sample_orientation = true;
    // cfg->goal_pos_tolerance = 3.0;
    cfg->start_connect_ratio = start_sample;
    cfg->steer_step = 16.0;
    // cfg->goal_bias = 0.0;
    cfg->DefaultSetup();
    cfg->env->SetCostType(ImageEnvironment::CostType::PATH_LENGTH);

    std::cout << "Planning parameters: r " << cfg->rad_curv << " l " << cfg->ins_length << " phi " << cfg->ang_constraint_degree 
                << "\ncost " << cfg->env->CostTypeString() << " constrain goal " << constrain_goal_orientation << " dubins " << global_dubins << std::endl;


#ifdef HAVE_GLOBAL_VARIABLES
    global::aorrt_cost_w = 1.0; // cost map 10; length 1; clearance 10
#endif

    // file is defined in global_common.h
    std::ifstream fin;
    fin.open(goal_file);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + goal_file);
    }

    std::vector<Vec3> goals;
    Vec3 center;

    Str line;

    while (std::getline(fin, line)) {
        std::istringstream s(line);
        s >> center[0] >> center[1] >> center[2];

        if ((center - start_p).norm() < cfg->ins_length + cfg->goal_pos_tolerance) {
            goals.emplace_back(center);
        }
    }

    // start_q = Quat::FromTwoVectors(Vec3::UnitZ(), (goals[0] - start_p).normalized());

    using Scenario = PAORRTSpreadingScenario<RealNum>::Type;
    using State = typename Scenario::State;
    using Space = typename Scenario::Space;

    State start(start_q, start_p);
    Scenario scenario(cfg, start);
    MPT_LOG(INFO) << "start: " << start;

    scenario.validator().ProvideGoalPoints(goals);
    scenario.goal().ProvideGoalPoints(goals);

    if (!scenario.ValidProblem()) {
        throw std::runtime_error("Planning problem is not valid!");
    }

    using namespace unc::robotics::mpt;
    using namespace unc::robotics::nigh;
    using NN = nn_select<RealNum, Space>::type;
    static constexpr bool reportStats = true;

    if (cfg->multi_threading) {
        using Threads = hardware_concurrency;
        using Algorithm = NeedlePRRT<report_stats<reportStats>, NN, Threads, spreading, optimal>;

        Planner<Scenario, Algorithm> planner(scenario);
        planner.addStart(start);
        planner.setGoalBias(cfg->goal_bias);

        utils::Run<0>(planner, cfg);
        auto const& result = planner.resultWithTime();
        for (auto const& res : result) {
            std::cout << std::get<0>(res) << ", " << std::get<1>(res) << ", " << std::get<2>(res) << ", " << std::get<3>(res) << std::endl;
        }
    }
    else {
        using Threads = single_threaded;
        using Algorithm = NeedlePRRT<report_stats<reportStats>, NN, Threads, spreading, optimal>;

        Planner<Scenario, Algorithm> planner(scenario, cfg->seed);
        planner.addStart(start);
        planner.setGoalBias(cfg->goal_bias);
        MPT_LOG(INFO) << "using seed " << cfg->seed;

        utils::Run<0>(planner, cfg);
        auto const& result = planner.resultWithTime();
        for (auto const& res : result) {
            std::cout << std::get<0>(res) << ", " << std::get<1>(res) << ", " << std::get<2>(res) << ", " << std::get<3>(res) << std::endl;
        }
    }

    return 0;
}