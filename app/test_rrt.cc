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

    // needle parameter file path is defined in global_common.h, these are the default parameters for the needle
    auto [min_curve_rad, needle_diameter, insertion_length, angle_constraint_degree]
        = utils::ReadNeedleParameters(needle_parameter_file, true);

    if (argc > 5) {
        min_curve_rad = std::atoi(argv[5]);
    }

#ifdef HAVE_GLOBAL_VARIABLES
    global::needle_min_curve_rad = min_curve_rad;
    global::angle_constraint_degree = angle_constraint_degree;
#endif

    bool constrain_goal_orientation = false;
    Str suffix = "_rrt";
    int scan_number = 0;

    if (argc > 1) {
        constrain_goal_orientation = std::atoi(argv[1]);
    }



    if (argc > 6) {
        suffix = argv[6];
        suffix = "_" + suffix;
    }

    if (argc > 4) {
        scan_number = std::atoi(argv[4]);
        
        start_and_goal_file = "../data/input/remind_00" + std::to_string(scan_number) + "_start_and_goal_poses.txt";
        global_obstacle_file = "../data/input/remind_obstacles_00" + std::to_string(scan_number) + "_outline_shuffled.txt";
        goal_file = "../data/input/remind_00" + std::to_string(scan_number) + "_goal_regions.txt";
        suffix = suffix + "_remind_00" + std::to_string(scan_number);
    }


    ConfigPtr cfg(new ProblemConfig(constrain_goal_orientation,
                                    min_curve_rad,
                                    needle_diameter,
                                    insertion_length,
                                    angle_constraint_degree));

    if (argc > 2) {
        cfg->multi_threading = std::atoi(argv[2]);
    }

    if (argc > 3) {
        cfg->seed = std::atoi(argv[3]);
    }

    
    // start_and_goal_file is defined in global_common.h 
    auto [start_p, start_q, goal_p, goal_q] = utils::ReadStartAndGoal(start_and_goal_file);

    cfg->output_file_root = "../data/output/" + date_and_time + suffix;
    cfg->direct_connect_ratio = 1.0;
    cfg->goal_pos_tolerance = 1.0;
    cfg->steer_step = -1;
    cfg->goal_bias = 0.05;
    cfg->DefaultSetup();
    cfg->env->SetCostType(ImageEnvironment::CostType::PATH_LENGTH);
    std::cout << "Using cost: " << cfg->env->CostTypeString() << std::endl;

    cfg->env->AddToWhiteList(start_p, 3);
    cfg->env->SetWhiteList(true);

    using Scenario = Point2PointScenario<RealNum>::Type;
    using State = typename Scenario::State;
    using Space = typename Scenario::Space;

    State start(start_q, start_p);
    State goal(goal_q, goal_p);
    Scenario scenario(cfg, start, goal);

    MPT_LOG(INFO) << "start: " << start;
    MPT_LOG(INFO) << "goal: " << goal;

    // checks if start/goal pair are reasonable for the environment and limits of the needle
    if (!scenario.ValidProblem()) {
        throw std::runtime_error("Planning problem is not valid!");
    }

    using namespace unc::robotics::mpt;
    using namespace unc::robotics::nigh;
    using NN = nn_select<RealNum, Space>::type;
    static constexpr bool reportStats = true;

    if (cfg->multi_threading) {
        using Threads = hardware_concurrency;
        using Algorithm = NeedlePRRT<report_stats<reportStats>, NN, Threads>;

        Planner<Scenario, Algorithm> planner(scenario);
        planner.addStart(start);
        planner.setGoalBias(cfg->goal_bias);

        utils::Run<0>(planner, cfg);

        auto const& result = planner.resultWithTime();
        for (auto const& res : result) {
            std::cout << res.first << ", " << res.second << std::endl;
        }
    }
    else {
        using Threads = single_threaded;
        using Algorithm = NeedlePRRT<report_stats<reportStats>, NN, Threads>;

        Planner<Scenario, Algorithm> planner(scenario, cfg->seed);
        planner.addStart(start);
        planner.setGoalBias(cfg->goal_bias);
        MPT_LOG(INFO) << "using seed " << cfg->seed;

        utils::Run<0>(planner, cfg);

        auto const& result = planner.resultWithTime();
        for (auto const& res : result) {
            std::cout << res.first << ", " << res.second << std::endl;
        }
    }

    return 0;
}