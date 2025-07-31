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
#ifndef SNP_NEEDLE_VALIDATOR_H
#define SNP_NEEDLE_VALIDATOR_H

#include <cmath>
#include <queue>

#include "../problem_config.h"
#include "../needle_utils.h"

namespace unc::robotics::snp {

namespace utils {

/**
 * checks if the workspace between the start (s) and goal are connected and within the limits of the needle uses the 
 * intersection of the trumpet shaped workspace created by the radius of curvature limit and the rugby/olive shaped 
 * workspace created by the points from which the goal is still reachable given the start
 * 
 * @param s: the starting state for the motion planning problem
 * @param goal: the goal state for the motion planning problem
 * @param rad_curv: the radius of curvature limit (?) for the needle
 * @param pos_tolerance: the postion tolerance used for saying not fully reached but "close enough"
 * @param neig: the index increments/decrements to visit a voxel's neighboring voxels for checks
 * @param env: the environment of the motion planning problem
 * @param visited: the voxels visited during the check
 * @param max_size: the maximum number of voxels to visit, used to stop early if things are connected up to that point
 * 
 * @returns bool true if the workspace is connected to a certain point
 */
template <typename State, bool Init=false>
bool CheckWorkspaceConnected(const State& s, const State& goal, const RealNum& rad_curv, const RealNum& pos_tolerance,
                             const std::vector<IntPoint>& neig, EnvPtr env, BoolArray3& visited, unsigned& max_size){
    const Vec3& sp = s.translation();
    const Quat sq = s.rotation().normalized();
    const Vec3 st = (sq * Vec3::UnitZ()).normalized();
    const Vec3& gp = goal.translation();
    const Vec3 sg = gp - sp;
    const Vec3 unit_sg = sg.normalized();
    const RealNum d = sg.norm();
    const RealNum& voxel_rad = env->VoxelRadius();
    const RealNum y = sg.dot(st);


    // std::cout << "st: " << st[0] << " " << st[1] << " " << st[2] << std::endl;
    // std::cout << "sg: " << sg[0] << " " << sg[1] << " " << sg[2] << std::endl;
    // std::cout << "y: " << y << std::endl;

    RealNum max_h;
    if (2 * rad_curv - pos_tolerance < d) {
        max_h = d + pos_tolerance;
    }
    else {
        RealNum r = rad_curv - pos_tolerance;
        RealNum cos_theta = (d*d + std::pow(rad_curv, 2) - r*r)/(2*d*rad_curv);
        max_h = 2 * rad_curv*cos_theta;
    }

    RealNum center_x;
    RealNum center_y = 0.5 * max_h;
    if (rad_curv > center_y) {
        center_x = std::sqrt(rad_curv * rad_curv - center_y * center_y);
    }
    else {
        center_x = 0;
    }
    const Vec2 rugby_center(-center_x, center_y);
    const RealNum rugby_rad = std::max(rad_curv, center_y);

    if constexpr (Init) {
        env->SetWorkspace(false);
    }
    std::fill(visited.data(), visited.data() + visited.num_elements(), false);

    bool connected = false;
    std::queue<IntPoint> queue;
    auto const start_ijk = env->RasToIjk(sp).cast<int>();   // TODO: has something like this changed behavior in debugging?

    // std::cout << "start ijk: " << start_ijk[0] << " " << start_ijk[1] << " " << start_ijk[2] << " voxel rad: " << voxel_rad <<std::endl;
    queue.push(start_ijk);
    if constexpr (Init) {
        env->SetWorkspace(start_ijk[0], start_ijk[1], start_ijk[2]);
    }
    visited[start_ijk[0]][start_ijk[1]][start_ijk[2]] = true;

    IntPoint cur_ijk, inc;
    IdxPoint inc_ijk;
    SizeType counter = 0;
    // std::cout << "max size: " << max_size << " init: " << Init << std::endl;
    while (!queue.empty()) {
        cur_ijk = queue.front();
        queue.pop();
        ++counter;
        // if (counter % 1000 == 0)
        // {
        //     std::cout << "\rChecked points: " << counter << std::flush;
        // }
        
        // std::cout << "counter: " << counter << " ijk: " << cur_ijk[0] << " " << cur_ijk[1] << " " << cur_ijk[2] << std::endl;
        if constexpr (!Init) {
            if (counter > max_size) {
                return true;
            }
        }

        // if (counter > 1000000) {
        //         return true;
        // }

        for (auto const& n : neig) {
            inc = cur_ijk + n;
            inc_ijk = inc.cast<Idx>();
            if (!env->WithinImage(inc_ijk)) {
                continue;
            }

            if (visited[inc[0]][inc[1]][inc[2]]) {
                continue;
            }
            visited[inc[0]][inc[1]][inc[2]] = true;

            bool valid = true;
            if constexpr (Init) {
                valid = !env->IsObstacle(inc_ijk);
            }
            else {
                valid = env->Workspace(inc[0], inc[1], inc[2]);
            }

            if (valid) {
                bool valid = false;
                const Vec3 inc_ras = env->IjkToRas(inc_ijk);
                if ((inc_ras - sp).norm() < voxel_rad) {
                    valid = true;
                }
                else {
                    RealNum dist_to_trumpet = DistanceToTrumpetBoundary(sp, st, inc_ras, rad_curv);
                    if (dist_to_trumpet < voxel_rad) {
                        const Vec3 relative_p = inc_ras - sp;
                        RealNum inc_y = relative_p.dot(unit_sg);
                        if (inc_y > -voxel_rad && inc_y < max_h + voxel_rad) {
                            RealNum inc_x = (relative_p - inc_y * unit_sg).norm();
                            if ((Vec2(inc_x, inc_y) - rugby_center).norm() < rugby_rad + voxel_rad) { // TODO: does this actually fully check the rugby? 
                                valid = true;
                            }
                        }
                    }
                }

                if (valid) {
                    if ((inc_ras - gp).norm() < voxel_rad + pos_tolerance) {
                        connected = true;
                        // std::cout << "counter: " << counter << std::endl;
                        if constexpr (!Init) {
                            // std::cout << std::endl;
                            return connected;
                        }
                    }
                    queue.push(inc);
                    if constexpr (Init) {
                        env->SetWorkspace(inc[0], inc[1], inc[2]);
                    }
                }
            }
        }
    }

    if constexpr (Init) {
        max_size = counter;
    }
    // std::cout << "counter: " << counter << std::endl;
    return connected;
}


/**
 * verifies that the problem is at least potentially possible to solve
 * 
 * @param start: the starting state for the motion planning problem
 * @param goal: the goal state for the motion planning problem
 * @param pos_tolerance: the postion tolerance used for saying not fully reached but "close enough"
 * @param ang_tolerance: 
 * @param neig: the index increments/decrements to visit a voxel's neighboring voxels for checks
 * @param env: the environment of the motion planning problem
 * @param rad_curv: the radius of curvature limit (?) for the needle
 * @param ins_length: the maximum insertion length
 * @param ang_constraint_rad: 
 * @param constrain_goal_orientation: 
 * @param visited: the voxels visited during the check
 * @param max_size: the maximum number of voxels to visit, used to stop early if things are connected up to that point
 * 
 * @returns bool true if the planning problem is valid with the current constraints 
 */
template <typename State>
bool ValidPoint2PointProblem(const State& start, const State& goal, const RealNum& pos_tolerance,
                             const RealNum& ang_tolerance, const std::vector<IntPoint>& neig,
                             EnvPtr env, const RealNum& rad_curv, const RealNum& ins_length,
                             const RealNum& ang_constraint_rad, const bool constrain_goal_orientation,
                             BoolArray3& visited, unsigned& max_size) {
    // if (ang_constraint_rad > 2*M_PI + EPS) {
    //     std::cout << "Using an angular constraint of " << ang_constraint_rad* RAD_TO_DEGREE <<
    //               " (> 270) degrees! Not supported yet!" << std::endl;
    //     return false;
    // }

    const Vec3& start_p = start.translation();
    const Quat start_q = start.rotation().normalized();
    const Vec3& goal_p = goal.translation();
    const Quat goal_q = goal.rotation().normalized();

    if (!env->WithinImage(start_p)) {
        std::cout << "Start point is outside image range!" << std::endl;
        return false;
    }

    if (!env->WithinImage(goal_p)) {
        std::cout << "Goal point is outside image range!" << std::endl;
        return false;
    }

    if (!env->CollisionFree(start_p)) {
        std::cout << "Start point is in collision!" << std::endl;
        return false;
    }

    if (!env->CollisionFree(goal_p)) {
        std::cout << "Goal point is in collision!" << std::endl;
        return false;
    }

    if ((start_p - goal_p).norm() > ins_length) {
        std::cout << "Exceed maximum insertion distance!" << std::endl;
        return false;
    }

    if (DistanceToTrumpetBoundary(start_p, start_q*Vec3::UnitZ(), goal_p, rad_curv) > pos_tolerance) {
        const RealNum max_rad = RadiusOfCurvature(start_p, start_q, goal_p, pos_tolerance);
        std::cout << "Goal point not in FRS, need rad to be " << max_rad << std::endl;
        return false;
    }

    if (!CheckWorkspaceConnected<State, true>(start, goal, rad_curv, pos_tolerance, neig, env, visited, max_size)) {
        std::cout << "Reachable workspace is not connected to the goal!" << std::endl;
        return false;
    }

    if (constrain_goal_orientation) {
        if (DistanceToTrumpetBoundary(goal_p, -(goal_q*Vec3::UnitZ()), start_p, rad_curv, ang_tolerance) > pos_tolerance) {
            const RealNum max_rad = RadiusOfCurvature(goal_p, -(goal_q*Vec3::UnitZ()), start_p, pos_tolerance);
            std::cout << "Start point not in BRS, need rad to be " << max_rad << std::endl;
            return false;
        }

        if (!WorkspaceConnected(start_p, start_q, goal_p, goal_q, rad_curv, pos_tolerance, ang_tolerance)) {
            std::cout << "Mission impossible (FRS and BRS not connected)!" << std::endl;
            return false;
        }
    }

    return true;
}


/**
 * Checks if the state exceeds the angle constraint from the start, does not account for path. 
 * 
 * @param s: the state s being considered
 * @param start: the starting state for the planner 
 * @param ang_constraint_rad: the maximum angle for the needle to follow (in radians)
 * 
 * @returns bool true if the angle between the z axes of s and start is greater than ang_constraint_rad
 */
template <typename State>
bool ExceedAngleConstraint(const State& s, const State& start, const RealNum ang_constraint_rad) {
    const RealNum diff = DirectionDifference(s.rotation(), start.rotation());

    return (diff > ang_constraint_rad);
}


/**
 * Checks that the state s is collision-free, in the limits of the trumpet workspace, and capable of reaching the goal state.
 * 
 * @param s: the state s being considered
 * @param start: the starting state for the planner 
 * @param goal: the goal state 
 * @param env: the planning environment
 * @param rad_curv: the minimum radius of curvature for the needle 
 * @param constrain_goal_orientation: should the planner try to reach a goal orientation
 * 
 * @returns bool true if the state passes all of the checks performed, false otherwise
 */
template <typename State>
bool ValidFullState(const State& s, const State& start, const State& goal, EnvPtr env,
                    const RealNum& rad_curv, const bool constrain_goal_orientation) {
    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized();

    if (!env->CollisionFree(p)) {
        return false;
    }

    if (!InTrumpet(start.translation(), start.rotation(), p, rad_curv)) {
        return false;
    }

    if (!InTrumpet(p, -(q*Vec3::UnitZ()), start.translation(), rad_curv)) {
        return false;
    }

    if (!InTrumpet(p, q, goal.translation(), rad_curv)) {
        return false;
    }

    if (constrain_goal_orientation) {
        if (!InTrumpet(goal.translation(), -(goal.rotation()*Vec3::UnitZ()), p, rad_curv)) {
            return false;
        }

        if (!WorkspaceConnected(p, q, goal.translation(), goal.rotation(), rad_curv)) {
            return false;
        }
    }

    return true;
}


/**
 * Checks that the state s is collision-free and capable of reaching the goal state.
 * 
 * @param s: the state s being considered
 * @param goal: the goal state
 * @param pos_tolerance: the tolerance for being "close enough" to the goal position
 * @param ang_tolerance: the orientation tolerance
 * @param env: the planning environment
 * @param rad_curv: the minimum radius of curvature for the needle
 * @param constrain_goal_orientation: should the planner try to reach a goal orientation
 * 
 * @returns bool true if the state passes all of the checks performed, false otherwise
 */
template <typename State>
bool ValidStateWithGoalReachability(const State& s, const State& goal, const RealNum& pos_tolerance,
                                    const RealNum& ang_tolerance,
                                    EnvPtr env, const RealNum& rad_curv, const bool constrain_goal_orientation) {
    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized();

    if (!env->CollisionFree(p)) {
        return false;
    }

    if (DistanceToTrumpetBoundary(p, q*Vec3::UnitZ(), goal.translation(),
                                         rad_curv) > pos_tolerance) {
        return false;
    }

    if (constrain_goal_orientation) {
        if (DistanceToTrumpetBoundary(goal.translation(), -(goal.rotation()*Vec3::UnitZ()), p,
                                             rad_curv, ang_tolerance) > pos_tolerance) {
            return false;
        }

        if (!WorkspaceConnected(p, q, goal.translation(), goal.rotation(), rad_curv, pos_tolerance,
                                ang_tolerance)) {
            return false;
        }
    }

    return true;
}


/**
 * Gets the curvature limit at the specific position and orientation given the configuraion.
 * 
 * @param sp: the position vector in world frame
 * @param sq: the quaternion representing the orientation in the world frame
 * @param cfg: the configuration for the planning problem
 * @param rad_curv: the global minimum feasible radius of curvature
 * 
 * @returns RealNum the local minimum radius of curvature for the orientation, greater than or equal to global minimum
 */
RealNum GetCurvature(const Vec3& sp, const Quat& sq, ConfigPtr cfg, const RealNum& rad_curv) {

    if (cfg->variable_curvature) {
        auto [skull_point, r_mag] = cfg->skull->NearestObstacleCenter(sp);
        skull_point = skull_point / 1000;
        Vec3 p = sp/1000;
        Vec3 r = Vec3(skull_point[0] - p[0], skull_point[1] - p[1], skull_point[2] - p[2]);
        Vec3 r_hat = r.normalized();
        r_mag = r.norm() + 0.020; // adding buffer for physical magnet radius
        Vec3 mag_point = Vec3(p[0] + r_hat[0]*(r_mag), p[1] + r_hat[1]*(r_mag), p[2] + r_hat[2]*(r_mag));
        auto r_outer = r_hat * r_hat.transpose(); // from https://stackoverflow.com/questions/74199536/computing-the-outer-product-of-two-vectors-in-eigen-c
        Vec3 needle_mag = sq.normalized() * Vec3::UnitZ();
        Vec3 x = sq.normalized() * Vec3::UnitX(); //r_hat.cross(s.rotation().normalized() * Vec3::UnitX()).normalized();//needle_mag.cross(r_hat);
        Vec3 manip_mag = sq.normalized() * Vec3::UnitY();

        auto r_mat = 3*r_outer - Eigen::Matrix3d::Identity();

        Vec3 b = cfg->manip_mag*(1e-7/(r_mag*r_mag*r_mag))*r_mat * manip_mag;
        Vec3 tau = cfg->needle_mag* needle_mag.cross(b);
        RealNum curvature_lim = 1/((cfg->torque_m*tau.norm() + cfg->torque_b)/1000);

        // std::cout << "r_outer: " << r_outer << std::endl;
        // // std::cout << "r_mat: " << r_mat << std::endl;
        // std::cout  << " curvature lim: " << curvature_lim << " |r|: " << r_mag << " max K: " << cfg->rad_curv << " |tau|: " << tau.norm();
        // std::cout  << " state: " << p[0] << " " << p[1] << " " << p[2] << " skull point: " << skull_point[0] << " " << skull_point[1] << " " << skull_point[2] << " r: " << r[0] << " " << r[1] << " " << r[2] << " mag point: " << mag_point[0] << " " << mag_point[1] << " " << mag_point[2] << std::endl;
        // std::cout << "manip: " << manip_mag.transpose() << " needle: " << needle_mag.transpose() << " y: " << y.transpose() << " q: " << sq.normalized() << std::endl;            
        // return curvature_lim;
        if (curvature_lim < cfg->rad_curv) {
            return cfg->rad_curv;
        } 
        else {
            // std::cout << "new limit: " << curvature_lim << std::endl;
            return curvature_lim;
        }
    }
    else {
        // std::cout << "lim: " << max_curvature_ << std::endl;
        return cfg->rad_curv;
    }

}


/**
 * Gets the curvature limit at the specific position and motion direction given the configuraion.
 * 
 * @param sp: the position vector in world frame
 * @param sq: the quaternion representing the orientation in the world frame, not of consequence to calcs here
 * @param normal_vec: the vector that the motion is about, and torque consequently
 * @param cfg: the configuration for the planning problem
 * @param rad_curv: the global minimum feasible radius of curvature
 * 
 * @returns RealNum the local minimum radius of curvature for the orientation, greater than or equal to global minimum
 */
RealNum GetCurvature(const Vec3& sp, const Quat& sq, const Vec3& normal_vec, ConfigPtr cfg, const RealNum& rad_curv) {

    if (cfg->variable_curvature) {
        auto [skull_point, r_mag] = cfg->skull->NearestObstacleCenter(sp);
        skull_point = skull_point / 1000;
        Vec3 p = sp/1000;
        // Vec3 test = Vec3(-68.224633, 18.392204, 74.247634);
        // auto [test_point, test_r] = cfg->skull->NearestObstacleCenter(test);
        
        // Vec3 test_diff = Vec3(test_point[0] - 1000*p[0], test_point[1] - 1000*p[1], test_point[2] - 1000*p[2]);
        // std::cout << "test: " << test_point[0] << " " << test_point[1] << " " << test_point[2] << " r: " << test_r << " diff: " << test_diff[0] << " " << test_diff[1] << " " << test_diff[2] << "r: " << test_diff.norm() << std::endl;

        Vec3 r = Vec3(skull_point[0] - p[0], skull_point[1] - p[1], skull_point[2] - p[2]);
        Vec3 r_hat = r.normalized();
        r_mag = r.norm() + 0.020; // adding buffer for physical magnet radius
        Vec3 mag_point = Vec3(p[0] + r_hat[0]*(r_mag), p[1] + r_hat[1]*(r_mag), p[2] + r_hat[2]*(r_mag));
        auto r_outer = r_hat * r_hat.transpose(); // from https://stackoverflow.com/questions/74199536/computing-the-outer-product-of-two-vectors-in-eigen-c
        Vec3 needle_mag = sq.normalized() * Vec3::UnitZ();
        Vec3 manip_mag = normal_vec;

        auto r_mat = 3*r_outer - Eigen::Matrix3d::Identity();

        Vec3 b = cfg->manip_mag*(1e-7/(r_mag*r_mag*r_mag))*r_mat * manip_mag;
        Vec3 tau = cfg->needle_mag* needle_mag.cross(b);
        RealNum curvature_lim = 1/((cfg->torque_m*tau.norm() + cfg->torque_b)/1000);

        // std::cout << "r_outer: " << r_outer << std::endl;
        // std::cout << "r_mat: " << r_mat << std::endl;

        // std::cout  << " curvature lim: " << curvature_lim << " |r|: " << r_mag << " max K: " << cfg->rad_curv << " |tau|: " << tau.norm();
        // std::cout  << " state: " << p[0] << " " << p[1] << " " << p[2] << " skull point: " << skull_point[0] << " " << skull_point[1] << " " << skull_point[2] << " r: " << r[0] << " " << r[1] << " " << r[2] << " mag point: " << mag_point[0] << " " << mag_point[1] << " " << mag_point[2] << std::endl;
        // std::cout << "manip: " << manip_mag.transpose() << " needle: " << needle_mag.transpose() << " y: " << y.transpose() << " q: " << sq.normalized() << std::endl;  
 
        
        // return curvature_lim;
        if (curvature_lim < cfg->rad_curv) {
            return cfg->rad_curv;
        } 
        else {
            // std::cout << "new limit: " << curvature_lim << std::endl;
            return curvature_lim;
        }
    }
    else {
        // std::cout << "lim: " << max_curvature_ << std::endl;
        return cfg->rad_curv;
    }

}


/**
 * Checks if the motion starting at from moving toward to is valid for the limits of the needle and the obstacles in the environment. 
 * @param from: starting state
 * @param to: target state
 * @param env: environment to check against
 * @param rad_curv: radius of curvature minimum limit 
 * @param resolution: resolution to check along of the arc length of the path for collisions
 * @param cfg: the configuration for the planning problem
 * 
 * @returns bool true if the motion is valid and collision free, false otherwise
 */
template <typename State>
bool ValidMotion(const State& from, const State& to, EnvPtr env, const RealNum& rad_curv,
                 const RealNum& resolution, const ConfigPtr cfg) {
    const Vec3& sp = from.translation();
    const Vec3& gp = to.translation();
    const Vec3 relative_pos = gp - sp;
    const RealNum d = relative_pos.norm();
    const Vec3 sg_hat = relative_pos/d;

    if (d < EPS) {
        return true;
    }

    bool print_ = false;
    // if ((abs(gp[0] + 64.00000) < 1e-5) || (abs(gp[0] + 48.79248457) < 1e-5) || (abs(gp[0] + 49.55040879) < 1e-5) || (abs(gp[0] + 50.43421668) < 1e-5) || (abs(gp[0] + 50.97510645) < 1e-5) || (abs(gp[0] + 55.2979713) < 1e-5) || (abs(gp[0] + 56.01266512) < 1e-5)) {
    //     print_ = true;
    // }

    const Quat sq_normalized = from.rotation().normalized();
    const Quat gq_normalized = to.rotation().normalized();
    const Vec3 st = (sq_normalized*Vec3::UnitZ()).normalized();                                     // the z axis of the start
    const Vec3 gt = (gq_normalized*Vec3::UnitZ()).normalized();                                     // the z axis of the goal
    const RealNum cos_theta = (relative_pos.normalized()).dot(st);                                  // cosine of the angle between the start and goal orientation
    Vec3 result_p;
    Quat result_q;
    RealNum result_rad;

    if (print_) {
    std::cout << "\n\tverifying p: " << gp[0] << " " << gp[1] << " " << gp[2] << std::endl;
    }
    

    // checking that there aren't collisions along the path, if the path is relatively straight
    if (cos_theta > 1 - EPS) {
        for (RealNum l = resolution; l < d; l += resolution) {              
            result_p = sp + st * l;
            // TODO: check curvature along path
            if (!env->CollisionFree(result_p)) {
                if (print_) {
                    std::cout << "collision!" << std::endl;
                }
                return false;
            }

            if (cfg->variable_curvature) {
                // result_q = (proceed_quat*sq_normalized).normalized();
                result_rad = GetCurvature(result_p, sq_normalized, sq_normalized*Vec3::UnitX(), cfg, rad_curv);
                // std::cout << "rad: " << result_rad << " lim: " << rad_curv << std::endl;
                // if the "distance to the trumpet boundary" is "nonzero" return false
                if (DistanceToTrumpetBoundary(sp, st, result_p, result_rad) > EPS) {
                    return false;
                }
                // if (result_rad < rad_curv) {
                //     return false;
                // }
            }            
        }
        std::cout << "\tp: " << result_p[0] << " " << result_p[1] << " " << result_p[2] << " verified !!!!!!!!!!!!!!! straight traj" << std::endl;
        return true;
    }

    // vector orthogonal to the z axes of the start and goal states
    // const Vec3 normal_vec = (sg_hat.cross(st)).normalized();
    const Vec3 normal_vec = (st.cross(gt)).normalized();

    // if the orthogonal vector and the vector between the start and goal are orthogonal return false
    if (normal_vec.dot(relative_pos.normalized()) > EPS) {
        return false;
    }

    // if the "distance to the trumpet boundary" is "nonzero" return false
    if (DistanceToTrumpetBoundary(sp, st, gp, rad_curv) > EPS) {
        return false;
    }

    if (cfg->variable_curvature){
        result_rad = GetCurvature(gp, gq_normalized, -normal_vec, cfg, rad_curv);

        if (print_) {
            std::cout << "new ind: -1 rad: " << result_rad << " lim: " << cfg->rad_curv << " p: " << gp[0] << " " << gp[1] << " " << gp[2] << " normal: " << normal_vec[0] << " " << normal_vec[1] << " " << normal_vec[2];
            std::cout << " z: " << (gq_normalized*Vec3::UnitZ()).normalized()[0] << " " << (gq_normalized*Vec3::UnitZ()).normalized()[1] << " " << (gq_normalized*Vec3::UnitZ()).normalized()[2] << " y: " << (gq_normalized*Vec3::UnitY()).normalized()[0] << " " << (gq_normalized*Vec3::UnitY()).normalized()[1] << " " << (gq_normalized*Vec3::UnitY()).normalized()[2] << " x: " << (gq_normalized*Vec3::UnitX()).normalized()[0] << " " << (gq_normalized*Vec3::UnitX())[1] << " " << (gq_normalized*Vec3::UnitX())[2];
            std::cout << std::endl;
        }

        if (DistanceToTrumpetBoundary(sp, st, gp, result_rad) > EPS) {
            if (print_) {
                std::cout << "gp not reachable" << std::endl;
            }
            return false;
        } 

        result_rad = GetCurvature(sp, sq_normalized, -normal_vec, cfg, rad_curv);

        if (print_) {
            std::cout << "new ind: -1 rad: " << result_rad << " lim: " << cfg->rad_curv << " p: " << sp[0] << " " << sp[1] << " " << sp[2] << " normal: " << normal_vec[0] << " " << normal_vec[1] << " " << normal_vec[2];
            std::cout << " z: " << (sq_normalized*Vec3::UnitZ()).normalized()[0] << " " << (sq_normalized*Vec3::UnitZ()).normalized()[1] << " " << (sq_normalized*Vec3::UnitZ()).normalized()[2] << " y: " << (sq_normalized*Vec3::UnitY()).normalized()[0] << " " << (sq_normalized*Vec3::UnitY()).normalized()[1] << " " << (sq_normalized*Vec3::UnitY()).normalized()[2] << " x: " << (sq_normalized*Vec3::UnitX()).normalized()[0] << " " << (sq_normalized*Vec3::UnitX())[1] << " " << (sq_normalized*Vec3::UnitX())[2];
            std::cout << std::endl;
        }

        if (DistanceToTrumpetBoundary(sp, st, gp, result_rad) > EPS) {
            if (print_) {
                std::cout << "sp not reachable" << std::endl;
            }
            return false;
        }         
    }
   

    // the radius of one of the circles comprising the rugby/olive shape??
    const RealNum r = 0.5 * d / std::sin(std::acos(cos_theta));

    // normal_vec x st = (st x gt) x st = gt ????
    // idk wtf this is the center of 
    const Vec3 center_diff = r*(normal_vec.cross(st));
    // const Vec3 other_center_diff = r*(other_normal_vec.cross(st));
    const Vec3 center = sp + center_diff;
    const RealNum max_angle = DirectionDifference(sq_normalized, gq_normalized);//std::acos(((gp - center).normalized()).dot((sp - center).normalized()));
    const RealNum angle_step = resolution / r;
    int i = 0;

    if (print_) {
        std::cout << "angle: " << max_angle << " center: " << center[0] << " " << center[1] << " " << center[2] << " r: " << r << " diff: " << center_diff[0] << " " << center_diff[1] << " " << center_diff[2];
        // std::cout << " other diff: " << other_center_diff[0] << " " << other_center_diff[1] << " " << other_center_diff[2];
        std::cout << " |r|: " << center_diff.norm()  << " |center|: " << center.norm() <<std::endl;
    }

    for (RealNum ang = angle_step; ang < max_angle; ang += angle_step) {
        Quat proceed_quat(AngleAxis(ang, normal_vec));
        result_p = proceed_quat*(sp - center) + center;
        
        if (!env->CollisionFree(result_p)) {
            if (print_) {
                std::cout << "collision!! " << result_p[0] << " " << result_p[1] << " " << result_p[2] << std::endl;

            }
            return false;
        }

        if (cfg->variable_curvature) {
            result_q = (proceed_quat*sq_normalized).normalized();
            result_rad = GetCurvature(result_p, result_q, normal_vec, cfg, rad_curv);

            if (print_) {
                std::cout << "new ind: " << i << " angle: " << ang << " rad: " << result_rad << " lim: " << cfg->rad_curv << " p: " << result_p[0] << " " << result_p[1] << " " << result_p[2] << " normal: " << normal_vec[0] << " " << normal_vec[1] << " " << normal_vec[2];
                std::cout << " z: " << (result_q*Vec3::UnitZ()).normalized()[0] << " " << (result_q*Vec3::UnitZ()).normalized()[1] << " " << (result_q*Vec3::UnitZ()).normalized()[2] << " y: " << (result_q*Vec3::UnitY()).normalized()[0] << " " << (result_q*Vec3::UnitY()).normalized()[1] << " " << (result_q*Vec3::UnitY()).normalized()[2] << " x: " << (result_q*Vec3::UnitX()).normalized()[0] << " " << (result_q*Vec3::UnitX())[1] << " " << (result_q*Vec3::UnitX())[2];
                std::cout << std::endl;
            }

            
            // std::cout << "rad: " << result_rad << " lim: " << rad_curv << std::endl;
            // if the "distance to the trumpet boundary" is "nonzero" return false
            if (DistanceToTrumpetBoundary(sp, st, result_p, result_rad) > EPS) {
                if (print_) {
                    std::cout << "radius limit!! " << result_p[0] << " " << result_p[1] << " " << result_p[2] << std::endl;
                }
                
                return false;
            }
            i++;
            // if (result_rad < rad_curv) {
            //     return false;
            // }
        }


    }
    if (print_) {
        std::cout << "motion valid!!!\n\n\n" << std::endl;
    }
    return true;
}

/**
 * Kinda seems like a binary search of the motion vector or something to speed up collision checks.
 * @param new_base: starting state
 * @param motion: motion from starting state, composed of other states
 * @param env: environment to collision check against 
 * @param cfg: the configuration for the planning problem
 * @param motion_rad: radius of the motion
 * @param offset: the offset index for the motion vector, states below the offset are ignored?? default is 0
 * 
 * @returns bool true if the motion is collision free, false otherwise
 */
template <typename State>
bool ValidMotion(const State& new_base, const std::vector<State>& motion, EnvPtr env, const ConfigPtr cfg, const RealNum motion_rad, const unsigned& offset=0) {
    bool print_ = false;
    if (!env) {
        throw std::runtime_error("No image environment! Cannot check path validity!");
    }

    if (motion.size() == 0) {
        throw std::runtime_error("Path contains no states! Cannot check path validity!");
    }

    const Vec3& base_p = new_base.translation();
    const Quat& base_q = new_base.rotation().normalized();
    const Vec3 base_t = (base_q*Vec3::UnitZ()).normalized();

    std::queue<std::pair<SizeType, SizeType>> queue;
    // queue.emplace(0, motion.size());
    queue.emplace(offset, motion.size());

    Vec3 result_p;
    Quat result_q;
    RealNum result_rad;
    Vec3 normal_vec;
    Vec3 result_t;

    result_p = base_q * motion[motion.size()-1].translation() + base_p;
    if (!env->CollisionFree(result_p)) {
        return false;                                        
    }                              

    // if (abs(result_p[0] + 50.9382384) < 1e-2) {
    //     print_ = true;
    // }
    // if (motion_rad > 100) {
    //     print_ = true;
    // }
    if (cfg->variable_curvature) {

        result_q = (base_q*motion[motion.size()-1].rotation()).normalized();
        result_t = (result_q*Vec3::UnitZ());
        normal_vec = (((base_q*motion[0].rotation())*Vec3::UnitZ()).cross(result_t)).normalized();
        if (normal_vec.norm() < 1e-5) {
            normal_vec = (base_q*motion[0].rotation())*Vec3::UnitY();
        }
        if (print_) {
            std::cout << "offset " << offset << " len: " << motion.size() << " p: " << base_p[0] << " " << base_p[1] << " " << base_p[2] << " base: " << base_t[0] << " " << base_t[1] << " " << base_t[2] << " q: " << base_q;
            std::cout  << " final: " << result_t[0] << " " << result_t[1] << " " << result_t[2] << std::endl;
        }
    
        result_rad = GetCurvature(base_p, base_q, normal_vec, cfg, cfg->rad_curv);
        if (result_rad > motion_rad) {
            // std::cout << "radius not in curvature limits!!!!!! at base" << std::endl;
            return false;
        }
        if (print_) {
            std::cout << "new index -1  rad: " << result_rad << " lim: " << cfg->rad_curv << " motion: " << motion_rad << " p: " << base_p[0] << " " << base_p[1] << " " << base_p[2] << " normal: " << normal_vec[0] << " " << normal_vec[1] << " " << normal_vec[2] << " z: " << base_t[0] << " " << base_t[1] << " " << base_t[2] << " y: " << (base_q*Vec3::UnitY()).normalized()[0] << " " << (base_q*Vec3::UnitY()).normalized()[1] << " " << (base_q*Vec3::UnitY()).normalized()[2] << " x: " << (base_q*Vec3::UnitX()).normalized()[0] << " " << (base_q*Vec3::UnitX())[1] << " " << (base_q*Vec3::UnitX())[2];
            std::cout << std::endl;
        }        

        for (unsigned i = 0; i < motion.size(); i++) {
            result_p = base_q * motion[i].translation() + base_p;
            if (!env->CollisionFree(result_p)) {
                // std::cout << "collision! " << i << std::endl; 
                return false;                                    
            }                              

            if (cfg->variable_curvature) {

                result_q = (base_q*motion[i].rotation().normalized()).normalized();
                result_t = (result_q*Vec3::UnitZ()).normalized();
                result_rad = GetCurvature(result_p, result_q, normal_vec, cfg, cfg->rad_curv);

                if (result_rad > motion_rad) {
                    // std::cout << "radius not in curvature limits!!!!!! " << i << std::endl;
                    return false;
                }
                if (print_) {
                    std::cout << "new index " << i << "  rad: " << result_rad << " lim: " << cfg->rad_curv << " motion: " << motion_rad << " p: " << result_p[0] << " " << result_p[1] << " " << result_p[2] << " normal: " << normal_vec[0] << " " << normal_vec[1] << " " << normal_vec[2] << " z: " << result_t[0] << " " << result_t[1] << " " << result_t[2] << " y: " << (result_q*Vec3::UnitY()).normalized()[0] << " " << (result_q*Vec3::UnitY()).normalized()[1] << " " << (result_q*Vec3::UnitY()).normalized()[2] << " x: " << (result_q*Vec3::UnitX()).normalized()[0] << " " << (result_q*Vec3::UnitX())[1] << " " << (result_q*Vec3::UnitX())[2];
                    std::cout << " q: " << motion[i].rotation().normalized() << " translation: " << motion[i].translation()[0] << " " << motion[i].translation()[1] << " " << motion[i].translation()[2];
                    std::cout << std::endl;
                }
            }
        } // TODO continue printing out the state before the offset to see what's going wrong at -55.1047  13.8953  57.182
        if (print_)
        {
            std::cout << "motion valid!!!" << std::endl;
        }
        return true;
    }


    // while (!queue.empty()) {
    //     auto p = queue.front();
    //     queue.pop();

    //     SizeType middle = p.first + (p.second - p.first)/2;
    //     // std::cout << "new index " << middle << std::endl;

    //     result_p = base_q * motion[middle].translation() + base_p;
    //     if (!env->CollisionFree(result_p)) {
    //         return false;                                        
    //     }                              

    //     if (cfg->variable_curvature) {

    //         result_q = (base_q*motion[middle].rotation().normalized()).normalized();
    //         result_t = (result_q*Vec3::UnitZ()).normalized();
    //         result_rad = GetCurvature(result_p, result_q, normal_vec, cfg, cfg->rad_curv);

    //         if (result_rad > motion_rad) {
    //             std::cout << "radius not in curvature limits!!!!!!\n" << std::endl;
    //             return false;
    //         }
    //         std::cout << "new index " << middle << "  rad: " << result_rad << " lim: " << cfg->rad_curv << " motion: " << motion_rad << " p: " << result_p[0] << " " << result_p[1] << " " << result_p[2] << " normal: " << normal_vec[0] << " " << normal_vec[1] << " " << normal_vec[2] << " z: " << result_t[0] << " " << result_t[1] << " " << result_t[2] << " y: " << (result_q*Vec3::UnitY()).normalized()[0] << " " << (result_q*Vec3::UnitY()).normalized()[1] << " " << (result_q*Vec3::UnitY()).normalized()[2] << " x: " << (result_q*Vec3::UnitX()).normalized()[0] << " " << (result_q*Vec3::UnitX())[1] << " " << (result_q*Vec3::UnitX())[2] << " q: " << motion[middle].rotation().normalized() << " translation: " << motion[middle].translation()[0] << " " << motion[middle].translation()[1] << " " << motion[middle].translation()[2] << std::endl;
    //     }

    //     if (p.first < middle) {
    //         queue.emplace(p.first, middle);
    //     }

    //     if (middle+1 < p.second) {
    //         queue.emplace(middle+1, p.second);
    //     }
    // }
    // std::cout << "motion valid!!!" << std::endl;
    return true;
}

/**
 * Kinda seems like a binary search of the motion vector or something to speed up collision checks.
 * @param motion: motion from starting state, composed of other states
 * @param env: environment to collision check against 
 * @param cfg: the configuration for the planning problem
 * @param motion_rad: radius of the motion
 * @param offset: the offset index for the motion vector, states below the offset are ignored?? default is 0
 * 
 * @returns bool true if the motion is collision free, false otherwise
 */
template <typename State>
bool ValidMotion(const std::vector<State>& motion, EnvPtr env, const ConfigPtr cfg, const RealNum motion_rad, const unsigned& offset=0) {
    if (!env) {
        throw std::runtime_error("No image environment! Cannot check path validity!");
    }

    if (motion.size() == 0) {
        throw std::runtime_error("Path contains no states! Cannot check path validity!");
    }

    std::queue<std::pair<SizeType, SizeType>> queue;
    queue.emplace(offset, motion.size());

    Vec3 result_p;
    Quat result_q;
    RealNum result_rad;

    // std::cout << "offset: " << offset << " length: " << motion.size() << std::endl;

    // TODO: add checking base curvature limit for this function too?


    while (!queue.empty()) {
        auto p = queue.front();
        queue.pop();

        SizeType middle = p.first + (p.second - p.first)/2;

        if (!env->CollisionFree(motion[middle].translation())) {
            return false;
        }

        if (cfg->variable_curvature) {
            result_q = motion[middle].rotation().normalized();
            result_rad = GetCurvature(motion[middle].translation(), result_q, cfg, cfg->rad_curv);
            
            if (result_rad < motion_rad) {
                return false;
            }
            // std::cout << "rad: " << result_rad << " lim: " << cfg->rad_curv << " motion: " << motion_rad << " p: " << result_p[0] << " " << result_p[1] << " " << result_p[2] << " size: " << motion.size() << std::endl;
        }

        if (p.first < middle) {
            queue.emplace(p.first, middle);
        }

        if (middle+1 < p.second) {
            queue.emplace(middle+1, p.second);
        }
    }

    return true;
}

/**
 * Attempts to connect the start state directly to the state s, checking for collisions along the path.
 * @param s: target state to connect with
 * @param start: starting state
 * @param env: environment to collision check against
 * @param rad_curv: radius of curvature minimum limit
 * @param resolution: resolution to collision check the path along the arc length
 * @param pi_x: an additonal rotation of the target state??
 * 
 * @returns State the resulting state after connecting the two states if it's reachable and collision free
 */
template <typename State>
std::optional<State> DirectConnecting(const State& s, const State& start, EnvPtr env,
                                      const RealNum& rad_curv, const RealNum& resolution, const Quat& pi_x) {
    State result = start;

    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized()*pi_x;
    const Vec3 t = q*Vec3::UnitZ();

    if (!InTrumpet(p, t, start.translation(), rad_curv)) {
        return {};
    }

    const State start_state = ForwardTo<State>(p, q, start.translation(), rad_curv);
    const std::vector<State> states = Interpolate<State>(p, q, start.translation(), start_state.rotation(),
                                      rad_curv, resolution);

    for (auto const& state : states) {
        if (!env->CollisionFree(state.translation())) {
            return {};
        }
    }

    result.rotation() = ((start_state.rotation())*pi_x).normalized();
    return result;
}

/**
 * Attempts to connect the start state with the state s without checking for collisions along the path.
 * @param s: target state to connect with
 * @param start: starting state
 * @param env: environment to collision check against
 * @param rad_curv: radius of curvature minimum limit
 * @param resolution: resolution to collision check the path along the arc length (not used)
 * @param pi_x: an additonal rotation of the target state??
 * 
 * @returns State the resulting state after connecting the two states if it's reachable
 */
template <typename State>
std::optional<State> DirectConnectingWithoutCollisionCheck(const State& s, const State& start, EnvPtr env,
                                                           const RealNum& rad_curv, const RealNum& resolution, const Quat& pi_x) {
    State result = start;

    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized()*pi_x;
    const Vec3 t = q*Vec3::UnitZ();

    if (!InTrumpet(p, t, start.translation(), rad_curv)) {
        return {};
    }

    const State start_state = ForwardTo<State>(p, q, start.translation(), rad_curv);
    result.rotation() = ((start_state.rotation())*pi_x).normalized();
    return result;
}


/**
 * Checks if the insertion length viiolates the constraints of the needle.
 * @param l: the insertion length
 * @param max_l: the maximum insertion length limit for the needle
 * 
 * @returns bool true if the insertion length is within the limits of the needle, false otherwise
 */
inline bool ValidLength(const RealNum& l, const RealNum& max_l) {
    return (l < max_l);
}

/**
 * Calculates the maximum arc length to reach the goal connecting the start and goal with a single arc ignoring orientation. 
 * @param s: starting position
 * @param g: goal position
 * @param rad_curv: radius of curvature minimum limit
 * 
 * @returns RealNum the maximum arc length to reach the goal from the start
 */
RealNum MaxCurveLength(const Vec3& s, const Vec3& g, const RealNum& rad_curv) {
    const RealNum d = (g - s).norm();

    if (rad_curv == R_INF) {
        return d;
    }

    if (d > 2 * rad_curv) {
        return std::sqrt(2) * d - 2 * rad_curv + 0.5 * M_PI * rad_curv;
    }

    const RealNum& theta = 2 * (std::asin(0.5 * d / rad_curv));

    return theta * rad_curv;
}

/**
 * Checks if the goal is reachable with a buffer region. 
 * @param s: starting state
 * @param goals: goal positions
 * @param rad_curv: radius of curvature minimum limit
 * @param ins_length: insertion length maximum limit
 * @param pos_tolerance: position tolerance used as sphere radius around the goal
 * 
 * @returns bool true if one of the goal positions is reachable and within the insertion limits of the needle, false otherwise
 */
template<typename State>
bool GoalSpheresReachable(const State& s, const std::vector<Vec3>& goals, const RealNum& rad_curv,
                          const RealNum& ins_length, const RealNum& pos_tolerance) {
    const Vec3& sp = s.translation();
    const Vec3& st = (s.rotation().normalized())*Vec3::UnitZ();

    for (auto const& p : goals) {
        if (ShortestDistance(s, p, rad_curv, pos_tolerance) > ins_length) {
            continue;
        }

        if (DistanceToTrumpetBoundary(sp, st, p, rad_curv) < pos_tolerance) {
            return true;
        }
    }

    return false;
}

/**
 * 
 * @param min_length_step:
 * 
 * @returns RealNum
 */
RealNum QueryLS(const RealNum& min_length_step) {
    // These values are emprically determined.
    if (std::abs(min_length_step - 0.078125) < EPS) {
        return 1.007681;
    }
    else if (std::abs(min_length_step - 0.125) < EPS) {
        return 1.012314;
    }
    else if (std::abs(min_length_step - 0.5) < EPS) {
        return 1.049267;
    }
    else if (std::abs(min_length_step - 1.0) < EPS) {
        return 1.098859;
    }
    else if (std::abs(min_length_step - 1.25) < EPS) {
        return 1.123808;
    }
    else if (std::abs(min_length_step - 2.0) < EPS) {
        return 1.198380;
    }
    else if (std::abs(min_length_step - 2.5) < EPS) {
        return 1.248207;
    }

    throw std::runtime_error("Undefined minimum length step!");
    return -1;
}

} // namespace utils

template<typename State>
class ValidatorBase {
  public:
    ValidatorBase(const ConfigPtr cfg) : ins_length_(cfg->ins_length), variable_curvature_(cfg->variable_curvature)
    {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }
        env_ = cfg->env;

        if (variable_curvature_) {
            if (cfg->skull == nullptr) {
                throw std::runtime_error("Skull is not defined for variable curvature calculations");
            }
            skull_ = cfg->skull;
            needle_mag_ = cfg->needle_mag;
            manip_mag_ = cfg->manip_mag;
            torque_b_ = cfg->torque_b;
            torque_m_ = cfg->torque_m;
            
        }
        max_curvature_ = cfg->rad_curv;
    }
    ~ValidatorBase() = default;

    /**
     * Checks if the state is in collision with the environment.
     * @param s: the state to collision check
     * 
     * @returns bool true if the state is in collision, false otherwise
     */
    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }

    /**
     * Checks if the state is in collision with the environment.
     * @param p: the position to collision check
     * 
     * @returns bool true if the position is in collision, false otherwise
     */
    bool InCollision(const Vec3& p) const {
        return (!env_->CollisionFree(p));
    }

    /**
     * Checks if the insertion length violates the constraints of the needle.
     * @param l: the insertion length
     * 
     * @returns bool true if the insertion length is within the limits of the needle, false otherwise
     */
    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, ins_length_);
    }


    RealNum GetCurvature(const State& s) const {

        if (variable_curvature_) {
            auto [skull_point, r_mag] = skull_->NearestObstacleCenter(s.translation());
            skull_point = skull_point / 1000;
            Vec3 p = s.translation()/1000;
            Vec3 r = Vec3(skull_point[0] - p[0], skull_point[1] - p[1], skull_point[2] - p[2]);
            Vec3 r_hat = r.normalized();
            r_mag = r.norm() + 0.020; // adding buffer for physical magnet radius
            Vec3 mag_point = Vec3(p[0] + r_hat[0]*(r_mag), p[1] + r_hat[1]*(r_mag), p[2] + r_hat[2]*(r_mag));
            auto r_outer = r_hat * r_hat.transpose(); // from https://stackoverflow.com/questions/74199536/computing-the-outer-product-of-two-vectors-in-eigen-c
            Vec3 needle_mag = s.rotation().normalized() * Vec3::UnitZ();
            Vec3 manip_mag = s.rotation().normalized() * Vec3::UnitX(); //r_hat.cross(s.rotation().normalized() * Vec3::UnitX()).normalized();//needle_mag.cross(r_hat);
            // std::cout << "r_outer: " << r_outer << std::endl;
            auto r_mat = 3*r_outer - Eigen::Matrix3d::Identity();
            // std::cout << "r_mat: " << r_mat << std::endl;
            Vec3 b = manip_mag_*(1e-7/(r_mag*r_mag*r_mag))*r_mat * manip_mag;
            Vec3 tau = needle_mag_* needle_mag.cross(b);
            RealNum curvature_lim = 1/((torque_m_*tau.norm() + torque_b_)/1000);

            // std::cout << "|r|: " << r_mag << " max K: " << max_curvature_ << " curvature lim: " << curvature_lim << " |tau|: " << tau.norm() << std::endl;
            // std::cout << " skull point: " << skull_point[0] << " " << skull_point[1] << " " << skull_point[2] << " state: " << p[0] << " " << p[1] << " " << p[2] << " r: " << r[0] << " " << r[1] << " " << r[2] << " mag point: " << mag_point[0] << " " << mag_point[1] << " " << mag_point[2] << std::endl;
            // std::cout << "manip: " << manip_mag.transpose() << " needle: " << needle_mag.transpose() << std::endl;            
            // return curvature_lim;
            if (curvature_lim < max_curvature_) {
                return max_curvature_;
            } 
            else {
                // std::cout << "new limit: " << curvature_lim << std::endl;
                return curvature_lim;
            }
        }
        else {
            // std::cout << "lim: " << max_curvature_ << std::endl;
            return max_curvature_;
        }

    }
    // TODO: add function calculating variable curvature here, may just be a wrapper for implementation in configuration if too much info needs to be passed?

    // b = r_hat * r_hat' * manip_m;
    // tau = skew_needle_m * b;
    // max_curvature = (torque_m_ * ||tau|| + torque_b_)/1000
    const RealNum ins_length_;
    EnvPtr env_;
    EnvPtr skull_;
    bool variable_curvature_;
    RealNum needle_mag_;
    RealNum manip_mag_;
    RealNum torque_b_;
    RealNum torque_m_;
    RealNum max_curvature_ = 14;
};

template<typename State>
class Point2PointCurveValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    Point2PointCurveValidator(const ConfigPtr cfg, const State& start, const State& goal,
                              const unsigned radius_status=0)
        : ValidatorBase<State>(cfg)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_(start)
        , goal_(goal)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , radius_status_(radius_status)
    {
        start_.rotation().normalize();
        goal_.rotation().normalize();

        neig_.emplace_back(-1, 0, 0);
        neig_.emplace_back(1, 0, 0);
        neig_.emplace_back(0, -1, 0);
        neig_.emplace_back(0, 1, 0);
        neig_.emplace_back(0, 0, -1);
        neig_.emplace_back(0, 0, 1);
        auto const& img_size = base::env_->ImageSize();
        visited_.resize(boost::extents[img_size[0]][img_size[1]][img_size[2]]);
    }

    /**
     * Checks if the problem is valid by checking if the starting point and the goal position are within the limits of the needle. 
     * 
     * @returns true if the start is valid and can reach at least one of the goal positions, false otherwise
     */
    bool ValidProblem() {
        return utils::ValidPoint2PointProblem(start_, goal_, pos_tolerance_, ang_tolerance_, neig_, base::env_,
                                            rad_curv_, ins_length_, ang_constraint_rad_, constrain_goal_orientation_,
                                            visited_, max_size_);
    }

    /**
     * Checks if the state is valid and respects the limits of the needle.
     * @param s: the state to check
     * @param length: the accumulated insertion length of the state from the start, default is 0
     * @param ang_total: the accumulated angle of the state from the start, default is 0
     * 
     * @returns bool true if the state is not in collision, respects needle lims, and can reach at least one goal position, false otherwise
     */
    bool Valid(const State& s, const RealNum& length=0, const RealNum& ang_total=0) const {

        if (radius_status_ > 0 && utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (ang_total > ang_constraint_rad_) {
            return false;
        }

        if (base::InCollision(s)) {
            return false;
        }

        return utils::ValidStateWithGoalReachability(s, goal_, pos_tolerance_, ang_tolerance_, base::env_,
                rad_curv_, constrain_goal_orientation_);
    }

    /**
     * Kinda seems like a binary search of the motion vector or something to speed up collision checks.
     * @param from: starting state
     * @param motion: motion from starting state, composed of other states
     * @param offset: the offset index for the motion vector, states below the offset are ignored??
     * 
     * @returns bool true if the motion is collision free, false otherwise
     */
    bool ValidMotion(const State& from, const State& to, const ConfigPtr cfg) const {
        return utils::ValidMotion(from, to, base::env_, rad_curv_, validity_res_, cfg);
    }

  private:
    const bool constrain_goal_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const unsigned radius_status_;

    State start_;
    State goal_;
    std::vector<IntPoint> neig_;
    BoolArray3 visited_;
    unsigned max_size_;
};

template<typename State>
class SpreadingValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    SpreadingValidator(const ConfigPtr cfg, const State& start)
        : ValidatorBase<State>(cfg)
        , fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
    {
        start_.rotation().normalize();
        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    /**
     * Sets the goal positions for the spreading validator.
     * @param goals: goal positions to use
     */
    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    /**
     * Checks if the spreading problem is valid by checking the starting point and if at least one of the goal positions is within the limits of the needle. 
     * 
     * @returns true if the start is valid and can reach at least one of the goal positions, false otherwise
     */
    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();

        if (!base::env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (base::InCollision(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(start_, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            std::cout << "None of the goal points are reachable from current start pose!" << std::endl;
            return false;
        }

        std::cout << "Valid problem with " << goals_.size() << " goal spheres." << std::endl;
        return true;
    }

    /**
     * Checks if the state is valid and respects the limits of the needle.
     * @param s: the state to check
     * @param length: the accumulated insertion length of the state from the start, default is 0
     * @param ang_total: the accumulated angle of the state from the start, default is 0
     * 
     * @returns bool true if the state is not in collision, respects needle lims, and can reach at least one goal position, false otherwise
     */
    bool Valid(const State& s, const RealNum& length=0, const RealNum& ang_total=0) const {
        // if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_) || ang_total > ang_constraint_rad_) {
        //     return false;
        // }

        if (ang_total > ang_constraint_rad_) {
            return false;
        }

        if (base::InCollision(s)) {
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(s, goals_, rad_curv_, ins_length_ - length, pos_tolerance_)) {
            return false;
        }

        return true;
    }

    /**
     * Checks if the motion between the two states is valid.
     * @param from: the starting state
     * @param to: the target state
     * 
     * @returns bool true if the motion is in the limits of the needle and collision free
     */
    bool ValidMotion(const State& from, const State& to, const ConfigPtr cfg) const {
        return utils::ValidMotion(from, to, base::env_, rad_curv_, validity_res_, cfg);
    }

    /**
     * Attempts to connect from that starting state to the provided state.
     * @param s: the target state to connect to
     * 
     * @returns State the resulting state after connecting if it is reachable
     */
    std::optional<State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {};
        }

        return utils::DirectConnectingWithoutCollisionCheck(s, start_, base::env_, rad_curv_,
                validity_res_, rotate_pi_x_);
    }

  private:
    const bool fix_orientation_;
    const RealNum pos_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;

    State start_;
    Quat rotate_pi_x_;
    std::vector<Vec3> goals_;
};

template<typename State>
class MotionPrimitiveValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    MotionPrimitiveValidator(const ConfigPtr cfg, const State& start, const State& goal,
                             const unsigned radius_status=0)
        : ValidatorBase<State>(cfg)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_(start)
        , goal_(goal)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , radius_status_(radius_status)
    {
        start_.rotation().normalize();
        goal_.rotation().normalize();

        const Idx max_length_i = std::ceil(std::log2(cfg->delta_ell_max/cfg->delta_ell_min));
        const RealNum min_length_step = cfg->delta_ell_max/(std::pow(2, max_length_i));
        max_insertion_ = utils::MaxCurveLength(start_.translation(), goal_.translation(), rad_curv_) + pos_tolerance_;
        const Idx max_depth = std::ceil(max_insertion_/min_length_step);

        if (max_depth == 0) {
            throw std::runtime_error("[ERROR] Max depth cannot be 0!");
        }

        RealNum ls = utils::QueryLS(min_length_step);
        RealNum delta = 0.5 * pos_tolerance_;

        if (cfg->optimal) {
            delta = std::min(delta, cfg->cost_approx_factor / base::env_->CostK());
        }

        config_tolerance_ = std::min(2 * rad_curv_ * std::sin(0.5 * min_length_step / rad_curv_),
                            SE3_T_W * 0.5 * delta * (ls - 1) / (std::pow(ls, max_depth) - 1)) - EPS;

        MPT_LOG(INFO) << "Config tolerance: " << config_tolerance_;

        goal_min_clearance_ = base::env_->DistanceToObstacleCenter(goal_.translation());

        neig_.emplace_back(-1, 0, 0);
        neig_.emplace_back(1, 0, 0);
        neig_.emplace_back(0, -1, 0);
        neig_.emplace_back(0, 1, 0);
        neig_.emplace_back(0, 0, -1);
        neig_.emplace_back(0, 0, 1);

        auto const& img_size = base::env_->ImageSize();
        visited_.resize(boost::extents[img_size[0]][img_size[1]][img_size[2]]);
    }

    /**
     * Checks if the problem is valid by checking if the starting point and the goal position are within the limits of the needle. 
     * 
     * @returns true if the start is valid and can reach at least one of the goal positions, false otherwise
     */
    bool ValidProblem() {
        const bool& valid = utils::ValidPoint2PointProblem(start_, goal_, pos_tolerance_, ang_tolerance_, neig_, base::env_,
                                            rad_curv_, ins_length_, ang_constraint_rad_, constrain_goal_orientation_,
                                            visited_, max_size_);
        max_size_ /= 10;
        return valid;
    }

    /**
     * Checks if the state is valid and respects the limits of the needle.
     * @param s: the state to check
     * @param length: the accumulated insertion length of the state from the start, default is 0
     * @param ang_total: the accumulated angle of the state from the start, default is 0
     * 
     * @returns bool true if the state is not in collision, respects needle lims, and can reach at least one goal position, false otherwise
     */
    bool Valid(const State& s, const RealNum& length=0, const RealNum& ang_total=0) const {
        if (radius_status_ > 0 && utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (!base::ValidLength(length))
        {
            return false;
        }

        if (ang_total > ang_constraint_rad_) {
            return false;
        }

        if (base::InCollision(s)) {
            return false;
        }

        return utils::ValidStateWithGoalReachability(s, goal_, pos_tolerance_, ang_tolerance_, base::env_,
                rad_curv_, constrain_goal_orientation_);
    }

    /**
     * Checks if the workspace between the state and goal are connected up to some point.
     * @param s: the state to use as the starting state for the worksapce calculations
     * 
     * @returns bool true if the workspace is connect to a point and the goal is reachable given the starting state
     */
    bool ValidReachableSpace(const State& s) {
        return utils::CheckWorkspaceConnected(s, goal_, rad_curv_, pos_tolerance_, neig_, base::env_, visited_, max_size_);
    }

    /**
     * Kinda seems like a binary search of the motion vector or something to speed up collision checks.
     * @param from: starting state
     * @param motion: motion from starting state, composed of other states
     * @param offset: the offset index for the motion vector, states below the offset are ignored??
     * 
     * @returns bool true if the motion is collision free, false otherwise
     */
    bool ValidMotion(const State& from, const std::vector<State>& motion, const ConfigPtr cfg, const RealNum motion_rad, const unsigned& offset) const {
        return utils::ValidMotion(from, motion, base::env_, cfg, motion_rad, offset);
    }

    /**
     * Kinda seems like a binary search of the motion vector or something to speed up collision checks.
     * @param motion: motion from starting state, composed of other states
     * 
     * @returns bool true if the motion is collision free, false otherwise
     */
    bool Valid(const std::vector<State>& motion) const {
        return utils::ValidMotion(motion, base::env_);
    }

    /**
     * Gets the configuration tolerance used to add a buffer on ...
     * 
     * @returns RealNum the configuration tolerance for the validator
     */
    const RealNum& ConfigTolerance() const {
        return config_tolerance_;
    }

    /**
     * Calculates the cost to get from the current state to the goal position, depends on the cost calculation settings of the environment.
     * @param s: the current state
     * 
     * @returns RealNum the cost to get from the current state to the goal, 1000 if it's out of the needle limits, 0 if the cost setting is not handled
     */
    RealNum CostToGo(const State& s) const {
        RealNum shortest_dist = ShortestDistance(s, goal_.translation(), rad_curv_, pos_tolerance_);

        if (shortest_dist == R_INF) {
            shortest_dist = 1000.0;
        }

        if (base::env_->ActiveCostType() == ImageEnvironment::CostType::PATH_LENGTH) {
            return shortest_dist;
        }

        if (base::env_->ActiveCostType() == ImageEnvironment::CostType::DIST_TO_OBS) {
            RealNum cur_clearance = base::env_->DistanceToObstacleCenter(s.translation());
            RealNum lower_bound = std::log(std::pow(cur_clearance
                                                    + goal_min_clearance_
                                                    + shortest_dist, 2)
                                           /(4*cur_clearance*goal_min_clearance_));
            return lower_bound;
        }

        if (base::env_->ActiveCostType() == ImageEnvironment::CostType::COST_MAP) {
            return base::env_->MinCost() * shortest_dist;
        }

        return 0;
    }

    /**
     * Calculates the IJK index of the state position.
     * @param s: the state to use in calculations
     * 
     * @returns IdxPoint IJK index of the state
     */
    IdxPoint ImageCoordinates(const State& s) const {
        return base::env_->RasToIjk(s.translation());
    }

    /**
     * Gets the goal state.
     * 
     * @returns State the goal state for the validator
     */
    const State& GoalState() const {
        return goal_;
    }

    /**
     * Gets the position tolerance for reachign the goal.
     * 
     * @returns RealNum the position tolerance for the validator
     */
    const RealNum& GoalTolerance() const {
        return pos_tolerance_;
    }

  private:
    const bool constrain_goal_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const unsigned radius_status_;
    RealNum config_tolerance_;
    RealNum max_insertion_;
    RealNum goal_min_clearance_;

    State start_;
    State goal_;
    std::vector<IntPoint> neig_;
    BoolArray3 visited_;
    unsigned max_size_;
};

template<typename State>
class MotionPrimitiveSpreadingValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    MotionPrimitiveSpreadingValidator(const ConfigPtr cfg, const State& start)
        : ValidatorBase<State>(cfg)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , healpix_file_(cfg->healpix_file)
    {
        start_.rotation().normalize();

        const Idx max_length_i = std::ceil(std::log2(cfg->delta_ell_max/cfg->delta_ell_min));
        const RealNum min_length_step = cfg->delta_ell_max/(std::pow(2, max_length_i));
        max_insertion_ = ins_length_;
        const Idx max_depth = std::ceil(max_insertion_/min_length_step);

        if (max_depth == 0) {
            throw std::runtime_error("[ERROR] Max depth cannot be 0!");
        }

        RealNum ls = utils::QueryLS(min_length_step);
        RealNum delta = 0.5 * pos_tolerance_;

        if (cfg->optimal) {
            delta = std::min(delta, cfg->cost_approx_factor / base::env_->CostK());
        }

        config_tolerance_ = std::min(2 * rad_curv_ * std::sin(0.5 * min_length_step / rad_curv_),
                            SE3_T_W * 0.5 * delta * (ls - 1) / (std::pow(ls, max_depth) - 1)) - EPS;

        MPT_LOG(INFO) << "Config tolerance: " << config_tolerance_;

        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    /**
     * Sets the goal positions for the spreading validator.
     * @param goals: goal positions to use
     */
    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    /**
     * Checks if the problem is valid by checking if the starting point and the goal position are within the limits of the needle. 
     * 
     * @returns true if the start is valid and can reach at least one of the goal positions, false otherwise
     */
    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();

        if (!base::env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (base::InCollision(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(start_, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            std::cout << "None of the goal points are reachable from current start pose!" << std::endl;
            return false;
        }

        std::cout << "Valid problem with " << goals_.size() << " goal spheres." << std::endl;
        return true;
    }

    /**
     * Checks if the state is valid and respects the limits of the needle.
     * @param s: the state to check
     * @param length: the accumulated insertion length of the state from the start, default is 0
     * @param ang_total: the accumulated angle of the state from the start, default is 0
     * 
     * @returns bool true if the state is not in collision, respects needle lims, and can reach at least one goal position, false otherwise
     */
    bool Valid(const State& s, const RealNum& length=0, const RealNum& ang_total=0) const {
        if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_) || ang_total > ang_constraint_rad_) {
            return false;
        }

        if (base::InCollision(s)) {
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(s, goals_, rad_curv_, ins_length_ - length, pos_tolerance_)) {
            return false;
        }

        return true;
    }

    /**
     * Kinda seems like a binary search of the motion vector or something to speed up collision checks.
     * @param from: starting state
     * @param motion: motion from starting state, composed of other states
     * @param offset: the offset index for the motion vector, states below the offset are ignored??
     * 
     * @returns bool true if the motion is collision free, false otherwise
     */
    bool ValidMotion(const State& from, const std::vector<State>& motion, const ConfigPtr cfg, const RealNum motion_rad, const unsigned& offset) const {
        return utils::ValidMotion(from, motion, base::env_, cfg, motion_rad, offset);
    }

    /**
     * Gets the configuration tolerance used to add a buffer on ...
     * 
     * @returns RealNum the configuration tolerance for the validator
     */
    const RealNum& ConfigTolerance() const {
        return config_tolerance_;
    }

    /**
     * Attempts to connect from that starting state to the provided state.
     * @param s: the target state to connect to
     * 
     * @returns State the resulting state after connecting if it is reachable
     */
    std::optional<State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {};
        }

        return utils::DirectConnectingWithoutCollisionCheck(s, start_, base::env_, rad_curv_,
                validity_res_, rotate_pi_x_);
    }

    /**
     * Initializes and loads the HEALPix environment. 
     */
    void InitHEALPix() {
        std::ifstream fin;
        fin.open(healpix_file_);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open " + healpix_file_);
        }

        Str line;
        Vec3 point;
        while (std::getline(fin, line)) {
            std::istringstream s(line);
            for (unsigned i = 0; i < 3; ++i) {
                s >> point[i];
            }

            healpix_vecs_.push_back(point);
        }

        MPT_LOG(INFO) << "Loaded " << healpix_vecs_.size() << " HEALPix points.";
    }

    /**
     * Gets the next starting state from the HEALPix data if one is available.
     * 
     * @returns the next starting state if one is available
     */
    std::optional<State> IterateNextStart() {
        if (healpix_idx_ >= healpix_vecs_.size()) {
            return {};
        }

        State res = start_;
        res.rotation() = Quat::FromTwoVectors(Vec3::UnitZ(), healpix_vecs_[healpix_idx_].normalized());
        healpix_idx_++;

        return res;
    }

  private:
    const bool constrain_goal_orientation_;
    const bool fix_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const Str healpix_file_;
    RealNum config_tolerance_;
    RealNum max_insertion_;

    State start_;
    std::vector<Vec3> goals_;
    Quat rotate_pi_x_;

    std::vector<Vec3> healpix_vecs_;
    SizeType healpix_idx_{0};
};

} // namespace unc::robotics::snp

#endif // SNP_NEEDLE_VALIDATOR_H