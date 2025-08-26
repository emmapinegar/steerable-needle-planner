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

    auto [min_curve_rad_, needle_diameter, insertion_length_, angle_constraint_degree_] = utils::ReadNeedleParameters(needle_parameter_file, false);

    auto suffix_ = "_aorrt";

    auto [constrain_goal_orientation, suffix, min_curve_rad, insertion_length, angle_constraint_degree] = ParseArgs(argc, argv, min_curve_rad_, insertion_length_, angle_constraint_degree_, suffix_);


    ConfigPtr cfg(new ProblemConfig(constrain_goal_orientation,
                                    min_curve_rad,
                                    needle_diameter,
                                    insertion_length,
                                    angle_constraint_degree));


    // start_and_goal_file is defined in global_common.h 
    auto [start_p_, start_q, goal_p_, goal_q] = utils::ReadStartAndGoal(start_and_goal_file);

    auto [start_p, goal_p] = utils::ReadSGPair(sg_pairs_file, global_sg_index);

    cfg->output_file_root = "../data/output/" + date_and_time + suffix;
    cfg->direct_connect_ratio = 1.0;
    // cfg->goal_pos_tolerance = 1.0;
    cfg->steer_step = 16.0;
    // cfg->goal_bias = 0.05;
    cfg->sample_orientation = true;
    cfg->optimal = true;
    cfg->DefaultSetup();
    cfg->env->SetCostType(ImageEnvironment::CostType::PATH_LENGTH);

    std::cout << "Planning parameters: r " << cfg->rad_curv << " l " << cfg->ins_length << " phi " << cfg->ang_constraint_degree 
                << "\ncost " << cfg->env->CostTypeString() << " constrain goal " << constrain_goal_orientation << " dubins " << global_dubins << std::endl;

#ifdef HAVE_GLOBAL_VARIABLES
    global::aorrt_cost_w = 1.0; // cost map 100; length 1; clearance 10;
#endif
    std::cout << "Using cost: " << cfg->env->CostTypeString() << std::endl;

    cfg->env->AddToWhiteList(start_p, 3);
    cfg->env->SetWhiteList(true);

    using Scenario = PAORRTPoint2PointScenario<RealNum>::Type;
    using State = typename Scenario::State;
    using Space = typename Scenario::Space;

    State start(start_q, start_p);
    State goal(goal_q, goal_p);
    Scenario scenario(cfg, start, goal);

    MPT_LOG(INFO) << "start: " << start;
    MPT_LOG(INFO) << "goal: " << goal;

    if (!scenario.ValidProblem()) {
        throw std::runtime_error("Planning problem is not valid!");
    }

    using namespace unc::robotics::mpt;
    using namespace unc::robotics::nigh;
    using NN = nn_select<RealNum, Space>::type;
    static constexpr bool reportStats = true;

    if (cfg->multi_threading) {
        using Threads = hardware_concurrency;
        using Algorithm = NeedlePRRT<report_stats<reportStats>, NN, Threads, optimal>;

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
        using Algorithm = NeedlePRRT<report_stats<reportStats>, NN, Threads, optimal>;

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