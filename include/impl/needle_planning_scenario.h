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
#ifndef SNP_NEEDLE_PLANNING_SCENARIO_H
#define SNP_NEEDLE_PLANNING_SCENARIO_H

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <mpt/box_bounds.hpp>
#include <mpt/log.hpp>
#include <mpt/uniform_sampler.hpp>

#include "../problem_config.h"
#include "goals/needle_goal.h"
#include "goals/needle_spreading_goal.h"
#include "needle_space.h"
#include "needle_sampler.h"
#include "needle_validator.h"
#include "needle_propagator.h"

namespace unc::robotics::snp {

template <typename DistanceSpace, typename PoseSampler, typename StatePropagator, typename StateValidator, unsigned Mode>
class NeedlePlanningScenario {};

template <typename DistanceSpace, typename PoseSampler, typename StatePropagator, typename StateValidator>
class NeedlePlanningScenario<DistanceSpace, PoseSampler, StatePropagator, StateValidator, 0> {
  public:
    using Space = DistanceSpace;
    using CSampler = PoseSampler;
    using Propagator = StatePropagator;
    using Validator = StateValidator;
    using Scalar = typename Space::Distance;
    using Distance = typename Space::Distance;
    using State = typename Space::Type;
    using Goal = mpt::NeedleGoalState<Space>;
    using Bounds = std::tuple<mpt::Unbounded, mpt::BoxBounds<Scalar, 3>>;
    using Position = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;

  private:
    Space space_;
    Bounds bounds_;
    const unsigned radius_status_;
    Validator validator_;
    Goal goal_;

    State start_;
    State goal_state_;
    ConfigPtr cfg_;

  public:
    NeedlePlanningScenario(ConfigPtr cfg, const State& start, const State& goal)
        : bounds_(this->MakeBounds(start, goal, cfg->ins_length))
        , radius_status_(this->CheckCurvatureStatus(start, goal, cfg))
        , validator_(cfg, start, goal, radius_status_)
        , goal_(cfg, goal)
        , start_(start)
        , goal_state_(goal)
        , cfg_(cfg) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of scenario failed! Config class doesn't have a valid environment!");
        }

        start_.rotation().normalize();
    }

    /**
     * Makes the bounds for the planning problem based off of the starting position and needle limits.
     * @param start: starting state of the needle
     * @param goal: goal state for the needle
     * @param length: needle insertion limit
     * 
     * @returns Bounds start +/- maximum insertion length in every direction
     */
    Bounds MakeBounds(const State& start, const State& goal, const Scalar& length) const {
        const Vec3& start_p = start.translation();
        const Vec3& goal_p = goal.translation();

        Position min, max;

        for (unsigned i = 0; i < 3; ++i) {
            min[i] = std::fmax(start_p[i], start_p[i]) - length;
            max[i] = std::fmin(start_p[i], start_p[i]) + length;
        }

        return Bounds(mpt::Unbounded{}, mpt::BoxBounds<Scalar, 3>(min, max));
    }

    /**
     * Gets the curvature status for the planning problem based on the distance between the start and goal
     * and where it falls with the different configuration thresholds. 
     * @param start: starting state of the needle
     * @param goal: goal state for the needle
     * @param cfg: planning configuration
     * 
     * @returns unsigned curvature status ( 0 -> d < threshold 0; 1 -> d < threshold 1; 2 -> d > threshold 0 & 1)
     */
    unsigned CheckCurvatureStatus(const State& start, const State& goal, const ConfigPtr cfg) {
        const Vec3& start_p = start.translation();
        const Vec3& goal_p = goal.translation();

        const RealNum d = (start_p - goal_p).norm();

        unsigned status;

        if (d < cfg->dist_threshold_0) {
            status = 0;
        }
        else if (d < cfg->dist_threshold_1) {
            status = 1;
        }
        else {
            status = 2;
        }

        return status;
    }

    /**
     * Checks with the validator if the scenario is valid.
     * 
     * @returns bool true if the problem is valid, false otherwise
     */
    bool ValidProblem() {
        return validator_.ValidProblem();
    }

    /**
     * Gets the planning configuration for the scenario.
     * 
     * @returns ConfigPtr the planning configuration
     */
    ConfigPtr Config() const {
        return cfg_;
    }

    /**
     * Gets the start state for the planning problem.
     * 
     * @returns State the start state
     */
    State StartState() const {
        return start_;
    }

    /**
     * Gets the goal state for the planning problem.
     * 
     * @returns State the goal state
     */
    State GoalState() const {
        return goal_.state();
    }

    /**
     * Checks with the validator if the state is valid in the planning context.
     * @param s: the state to check
     * 
     * @returns bool true if the state is valid, false otherwise
     */
    bool valid(const State& s) const {
        return validator_.Valid(s);
    }

    /**
     * Checks with the validator if the length is valid in the planning context.
     * @param length: the insertion length of a state
     * 
     * @returns bool true if the length is valid, false otherwise
     */
    bool valid(const Distance& length) const {
        return validator_.ValidLength(length);
    }

    /**
     * Checks with the validator if the state and length are valid independently.
     * @param s: the state to check
     * @param length: the insertion length of state
     * 
     * @returns true if the insertion length and state are valid, false otherwise
     */
    bool valid(const State& s, const Distance& length) const {
        return (validator_.ValidLength(length) && validator_.Valid(s));
    }

    /**
     * Checks with the validator if the state, length, and angle are valid.
     * @param s: the state to check
     * @param length: the insertion length of state
     * @param angle: the accumulated angle of the state from the start
     * 
     * @returns bool true if the state, length, and angle are all valid, false otherwise
     */
    bool valid(const State& s, const Distance& length, const Scalar& angle) const {
        return validator_.Valid(s, length, angle);
    }

    /**
     * Checks with the validator if the state is in the valid reachable workspace for the planning problem.
     * @param s: the state to check
     * 
     * @returns bool true if the state is in the valid reachable space, false otherwise
     */
    bool validReachableSpace(const State& s) {
        return validator_.ValidReachableSpace(s);
    }

    /**
     * Checks with the validator if the state is in collision.
     * @param s: the state to collision check
     * 
     * @returns bool true if the state is in collision, false otherwise
     */
    bool collision(const State& s) const {
        return validator_.InCollision(s);
    }

    /**
     * Checks with the validator if there is a valid motion between the two states.
     * @param from: starting state
     * @param to: target state
     * 
     * @returns bool true if there is a valid motion between the states, false otherwise
     */
    bool link(const State& from, const State& to) const {
        return validator_.ValidMotion(from, to);
    }

    /**
     * Calculates the euclidean distance between the two state positions.
     * @param s1: first state
     * @param s2: second state
     * 
     * @returns Scalar the euclidean distance between s1 and s2
     */
    Scalar PositionDist(const State& s1, const State& s2) const {
        return (s1.translation() - s2.translation()).norm();
    }

    /**
     * Gets the cost of the curve between the states from the configuration environment.
     * @param s1: starting state
     * @param s2: target state
     * 
     * @returns Scalar the cost of the curve 
     */
    Scalar CurveCost(const State& s1, const State& s2) const {
        return cfg_->env->CurveCost(s1.translation(), s1.rotation(), s2.translation(), s2.rotation(),
                                    cfg_->rad_curv, cfg_->cost_res);
    }

    /**
     * Gets the cost to get to the goal state from the current state from the configuration environment.
     * @param s: the current state
     * 
     * @returns Scalar cost to get from s to the goal state
     */
    Scalar FinalStateCost(const State& s) const {
        return cfg_->env->FinalStateCost(s.translation(), s.rotation(),
                                         goal_state_.translation(), goal_state_.rotation());
    }

    /**
     * Gets the planning space.
     * 
     * @returns Space the planning space
     */
    const Space& space() const {
        return space_;
    }

    /**
     * Gets the planning bounds.
     * 
     * @returns Bounds the bounds of the planning problem
     */
    const Bounds& bounds() const {
        return bounds_;
    }

    /**
     * Gets the goal.
     * 
     * @returns Goal goal for the planning problem
     */
    const Goal& goal() const {
        // std::cout << "goal: " << goal_ << std::endl;
        return goal_;
    }

    /**
     * Gets the validator.
     * 
     * @returns Validator validator for the planning problem
     */
    const Validator& validator() const {
        return validator_;
    }
};

template <typename DistanceSpace, typename PoseSampler, typename StatePropagator, typename StateValidator>
class NeedlePlanningScenario<DistanceSpace, PoseSampler, StatePropagator, StateValidator, 1> {
  public:
    using Space = DistanceSpace;
    using CSampler = PoseSampler;
    using Propagator = StatePropagator;
    using Validator = StateValidator;
    using Scalar = typename Space::Distance;
    using Distance = typename Space::Distance;
    using State = typename Space::Type;
    using Goal = mpt::NeedleSpreadingGoal<Space>;
    using Bounds = std::tuple<mpt::Unbounded, mpt::BoxBounds<Scalar, 3>>;
    using Position = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;

  private:
    Space space_;
    Bounds bounds_;
    Validator validator_;
    Goal goal_;

    State start_;
    ConfigPtr cfg_;

  public:
    NeedlePlanningScenario(ConfigPtr cfg, const State& start)
        : bounds_(this->MakeBounds(start.translation(), cfg->ins_length))
        , validator_(cfg, start)
        , goal_(cfg, start.translation())
        , start_(start)
        , cfg_(cfg) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of scenario failed! Config class doesn't have a valid environment!");
        }

        start_.rotation().normalize();
    }

    /**
     * Makes the bounds for the planning problem based off of the starting position and needle limits.
     * @param start: starting state of the needle
     * @param length: needle insertion limit
     * 
     * @returns Bounds start +/- maximum insertion length in every direction
     */
    Bounds MakeBounds(const Position& start, const Scalar& length) const {
        Eigen::Matrix<Scalar, 3, 1> min, max;

        for (unsigned i = 0; i < 3; ++i) {
            min[i] = start[i] - length;
            max[i] = start[i] + length;
        }

        return Bounds(mpt::Unbounded{}, mpt::BoxBounds<Scalar, 3>(min, max));
    }

    /**
     * Checks with the validator if the scenario is valid.
     * 
     * @returns bool true if the problem is valid, false otherwise
     */
    bool ValidProblem() const {
        return validator_.ValidProblem();
    }

    /**
     * Gets the planning configuration for the scenario.
     * 
     * @returns ConfigPtr the planning configuration
     */
    ConfigPtr Config() const {
        return cfg_;
    }

    /**
     * Gets the start state for the planning problem.
     * 
     * @returns State the start state
     */
    State StartState() const {
        return start_;
    }

    /**
     * Attempts to connect to the state directly from the start using the validator.
     * @param s: the state to try to connect to 
     * 
     * @returns State the resulting state if the connection was successful 
     */
    std::optional<State> DirectConnectingStart(const State& s) const {
        return validator_.DirectConnectingStart(s);
    }

    /**
     * Checks with the validator if the state is valid in the planning context.
     * @param s: the state to check
     * 
     * @returns bool true if the state is valid, false otherwise
     */
    bool valid(const State& s) const {
        return validator_.Valid(s);
    }

    /**
     * Checks with the validator if the length is valid in the planning context.
     * @param length: the insertion length of a state
     * 
     * @returns bool true if the length is valid, false otherwise
     */
    bool valid(const Distance& length) const {
        return validator_.ValidLength(length);
    }

    /**
     * Checks with the validator if the state and length are valid independently.
     * @param s: the state to check
     * @param length: the insertion length of state
     * 
     * @returns true if the insertion length and state are valid, false otherwise
     */
    bool valid(const State& s, const Distance& length) const {
        return validator_.Valid(s, length);
    }

    /**
     * Checks with the validator if the state, length, and angle are valid.
     * @param s: the state to check
     * @param length: the insertion length of state
     * @param angle: the accumulated angle of the state from the start
     * 
     * @returns bool true if the state, length, and angle are all valid, false otherwise
     */
    bool valid(const State& s, const Distance& length, const Scalar& angle) const {
        return validator_.Valid(s, length, angle);
    }

    /**
     * Checks with the validator if the state is in collision.
     * @param s: the state to collision check
     * 
     * @returns bool true if the state is in collision, false otherwise
     */
    bool collision(const State& s) const {
        return validator_.InCollision(s);
    }

    /**
     * Checks with the validator if there is a valid motion between the two states.
     * @param from: starting state
     * @param to: target state
     * 
     * @returns bool true if there is a valid motion between the states, false otherwise
     */
    bool link(const State& from, const State& to) const {
        return validator_.ValidMotion(from, to);
    }

    /**
     * Calculates the euclidean distance between the two state positions.
     * @param s1: first state
     * @param s2: second state
     * 
     * @returns Scalar the euclidean distance between s1 and s2
     */
    Scalar PositionDist(const State& s1, const State& s2) const {
        return (s1.translation() - s2.translation()).norm();
    }

    /**
     * Gets the cost of the curve between the states from the configuration environment.
     * @param s1: starting state
     * @param s2: target state
     * 
     * @returns Scalar the cost of the curve 
     */
    Scalar CurveCost(const State& s1, const State& s2) const {
        return cfg_->env->CurveCost(s1.translation(), s1.rotation(), s2.translation(), s2.rotation(),
                                    cfg_->rad_curv, cfg_->cost_res);
    }

    /**
     * Gets the cost to get to the goal state from the current state from the configuration environment.
     * @param s: the current state
     * 
     * @returns Scalar cost to get from s to the goal state
     */
    Scalar FinalStateCost(const State& s) const {
        return 0;
    }

    /**
     * Gets the planning space.
     * 
     * @returns Space the planning space
     */
    const Space& space() const {
        return space_;
    }

    /**
     * Gets the planning bounds.
     * 
     * @returns Bounds the bounds of the planning problem
     */
    const Bounds& bounds() const {
        return bounds_;
    }

    /**
     * Gets the goal.
     * 
     * @returns Goal goal for the planning problem
     */
    Goal& goal() {
        return goal_;
    }

    /**
     * Gets the goal.
     * 
     * @returns const Goal goal for the planning problem
     */
    const Goal& goal() const {
        return goal_;
    }

    /**
     * Gets the validator.
     * 
     * @returns Validator validator for the planning problem
     */
    Validator& validator() {
        return validator_;
    }

    /**
     * Gets the validator.
     * 
     * @returns const Validator validator for the planning problem
     */
    const Validator& validator() const {
        return validator_;
    }
};

} // namespace unc::robotics::snp

#endif // SNP_NEEDLE_PLANNING_SCENARIO_H