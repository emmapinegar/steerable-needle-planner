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
    

    // needle parameter file path is defined in global_common.h, these are the default parameters for the needle
    auto [min_curve_rad_, needle_diameter, insertion_length_, angle_constraint_degree_]
        = utils::ReadNeedleParameters(needle_parameter_file, false);

    Str suffix_ = "_rrt";  
    
    auto [constrain_goal_orientation, suffix, min_curve_rad, insertion_length, angle_constraint_degree] = ParseArgs(argc, argv, min_curve_rad_, insertion_length_, angle_constraint_degree_, suffix_);


    ConfigPtr cfg(new ProblemConfig(constrain_goal_orientation,
                                    min_curve_rad,
                                    needle_diameter,
                                    insertion_length,
                                    angle_constraint_degree));

    
    // start_and_goal_file is defined in global_common.h 
    auto [start_p_, start_q, goal_p_, goal_q] = utils::ReadStartAndGoal(start_and_goal_file);

    std::mt19937_64 seed_rng;
    seed_rng.seed(global_seed);
    RealUniformDist seed_generator = RealUniformDist(0, 14227);
    

    
    cfg->direct_connect_ratio = 1.0;
    // cfg->goal_pos_tolerance = 1.0;
    cfg->steer_step = -1;
    // cfg->goal_bias = 0.05;
    cfg->DefaultSetup();
    cfg->env->SetCostType(ImageEnvironment::CostType::PATH_LENGTH);
    
    int max_sg = global_sg_index + global_sg_num;
    for (global_sg_index; global_sg_index < max_sg; global_sg_index++) {
        Str date_and_time = utils::DateAndTime();
        cfg->seed = seed_generator(seed_rng);
        cfg->output_file_root = "../data/output/" + date_and_time + suffix;
        auto [start_p, goal_p] = utils::ReadSGPair(sg_pairs_file, global_sg_index);

        if (global_variable_curvature)
        {
            std::cout << "variable curvature mode" << std::endl;
        }

        std::cout << "Planning parameters: r " << cfg->rad_curv << " l " << cfg->ins_length << " phi " << cfg->ang_constraint_degree 
                    << "\ncost " << cfg->env->CostTypeString() << " constrain goal " << constrain_goal_orientation << " dubins " << global_dubins << std::endl;

        // cfg->env->AddToWhiteList(start_p, 3);
        // cfg->env->SetWhiteList(true);

        using Scenario = Point2PointScenario<RealNum>::Type;
        using State = typename Scenario::State;
        using Space = typename Scenario::Space;

        State start(start_q, start_p);
        State goal(goal_q, goal_p);
        Scenario scenario(cfg, start, goal);
        global_sg_mag = (goal_p - start_p).norm();
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
            MPT_LOG(INFO) << "using seed " << cfg->seed;

            utils::Run<0>(planner, cfg);

            auto const& result = planner.resultWithTime();
            for (auto const& res : result) {
                std::cout << std::get<0>(res) << ", " << std::get<1>(res) << ", " << std::get<2>(res) << ", " << std::get<3>(res) << std::endl;
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
                std::cout << std::get<0>(res) << ", " << std::get<1>(res) << ", " << std::get<2>(res) << ", " << std::get<3>(res) << std::endl;
            }
        }
    }





    return 0;
}