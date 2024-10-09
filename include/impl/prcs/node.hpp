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
#ifndef MPT_IMPL_PRCS_NODE_HPP
#define MPT_IMPL_PRCS_NODE_HPP

#include "edge.hpp"
#include <utility>
#include <array>
#include <set>
#include <mutex>
#include <iostream>
#include <unordered_set>

namespace unc::robotics::mpt::impl::prcs {
using ResLevels = std::array<unsigned, 2>;
using NodeIndices = std::array<unsigned, 3>;

/**
 * Converts Idx to a string (idx[0],idx[1],idx[2]).
 * @param idx: index to convert
 * 
 * @returns Str version of index
 */
template <typename Vector>
Str IdxToStr(const Vector& idx) {
    return std::to_string(idx[0]) + ","
           + std::to_string(idx[1]) + ","
           + std::to_string(idx[2]);
}

using IndexSet3 = std::unordered_set<Str>;

/**
 * Nodes for the the motion planner to track states and path attributes.
 */
template <typename State, typename Traj>
class Node {
    using Scalar = typename State::Distance;

    State state_;
    Edge<State, Traj> parent_;
    Scalar traj_length_{0};
    Scalar cost_to_come_{0};
    Scalar cost_to_go_{0};
    Scalar ang_total_{0};
    bool valid_{false};

    unsigned rank_{0};
    ResLevels levels_{ {0,0} };
    NodeIndices indices_{ {0,0,0} };

    IndexSet3 explored_;
    std::mutex mutex_;

  public:
    /**
     * Creates a Node object for an RRT based planner.
     * @param traj: trajectory the node belongs to
     * @param parent: parent node 
     * @param args: ??
     * 
     * @returns Node node in the trajectory with the given parent node
     */
    template <typename ... Args>
    Node(Traj&& traj, Node* parent, Args&& ... args)
        : state_(std::forward<Args>(args)...)
        , parent_(std::move(traj), parent) {
    }

    /**
     * Resets the node (state, parent, costs, validity, rank, levels, indices, explored set).
     * @param traj: trajectory 
     * @param parent: parent node
     * @param args: extra args idk what they would be :(
     */
    template <typename ... Args>
    void reset(Traj&& traj, Node* parent, Args&& ... args) {
        state_ = State(std::forward<Args>(args)...);
        parent_ = Edge<State, Traj>(std::move(traj), parent);
        traj_length_ = 0;
        cost_to_come_ = 0;
        cost_to_go_ = 0;
        valid_ = false;
        rank_ = 0;
        levels_ = {0,0};
        indices_ = {0,0,0};
        explored_.clear();
    }

    /**
     * 
     * @param levels: levels for the node [length, angle]
     * @param indices: indices for the node [radius, length, angle]
     */
    void setResolution(const ResLevels& levels, const NodeIndices& indices) {
        rank_ = (((Node*)parent_) == nullptr ? 0 : ((Node*)parent_)->rank()) + levels[0] + levels[1] + 1;
        levels_ = levels;
        indices_ = indices;
    }

    /**
     * Gets the cumulative arc length of the trajectory up to this node.
     * 
     * @returns Scalar trajectory arc length
     */
    Scalar& length() {
        return traj_length_;
    }

    /**
     * Gets the cumulative arc length of the trajectory up to this node.
     * 
     * @returns const Scalar trajectory arc length
     */
    const Scalar& length() const {
        return traj_length_;
    }

    /**
     * Gets the cumulative cost of the trajectory up to this node.
     * 
     * @returns Scalar trajectory cost
     */
    Scalar& cost() {
        return cost_to_come_;
    }

    /**
     * Gets the cumulative cost of the trajectory up to this node.
     * 
     * @returns const Scalar trajectory cost
     */
    const Scalar& cost() const {
        return cost_to_come_;
    }

    /**
     * Gets the estimated cost to get to the goal from the current node.
     * 
     * @returns Scalar estimated cost from current node to goal
     */
    Scalar& costToGo() {
        return cost_to_go_;
    }

    /**
     * Gets the estimated cost to get to the goal from the current node.
     * 
     * @returns const Scalar estimated cost from current node to goal
     */
    const Scalar& costToGo() const {
        return cost_to_go_;
    }

    /**
     * Gets the cumulative angle of the trajectory up to this node.
     * 
     * @returns Scalar trajectory angle
     */
    Scalar& ang_total() {
        return ang_total_;
    }

    /**
     * Gets the cumulative angle of the trajectory up to this node.
     * 
     * @returns const Scalar trajectory angle
     */
    const Scalar& ang_total() const {
        return ang_total_;
    }  

    /**
     * Calculates the cost to come to the current state and the estimated cost to reach the goal. 
     * 
     * @returns Scalar cost to come + cost to go
     */
    Scalar f() const {
        return cost_to_come_ + cost_to_go_;
    }

    /**
     * Gets if the node is valid.
     * 
     * @returns bool true if the node is valid, false otherwise
     */
    bool& valid() {
        return valid_;
    }

    /**
     * Gets if the node is valid.
     * 
     * @returns bool true if the node is valid, false otherwise
     */
    const bool& valid() const {
        return valid_;
    }

    /**
     * Gets the node state.
     * 
     * @returns State of node 
     */
    State& state() {
        return state_;
    }

    /**
     * Gets the node state.
     * 
     * @returns const State of node 
     */
    const State& state() const {
        return state_;
    }

    /**
     * Gets the edge between the node and its parent.
     * 
     * @returns const Edge<State, Traj> connection between node and its parent 
     */  
    const Edge<State, Traj>& edge() const {
        return parent_;
    }

    /**
     * Gets the parent of the node.
     * 
     * @returns const Node parent of the current node
     */ 
    const Node* parent() const {
        return parent_;
    }

    /**
     * Gets the parent of the node.
     * 
     * @returns Node parent of the current node
     */ 
    Node* parent() {
        return parent_;
    }

    /**
     * Gets the length level of the node.
     * 
     * @returns const unsigned length level of the node
     */
    const unsigned& lengthLevel() const {
        return levels_[0];
    }

    /**
     * Gets the angle level of the node.
     * 
     * @returns const unsigned angle level of the node
     */
    const unsigned& angleLevel() const {
        return levels_[1];
    }

    /**
     * Gets the rank of the node.
     * 
     * @returns const unsigned rank of the node
     */
    const unsigned& rank() const {
        return rank_;
    }

    /**
     * Gets the radius index of the node.
     * 
     * @returns const unsigned radius index of the node
     */
    const unsigned& radIndex() const {
        return indices_[0];
    }

    /**
     * Gets the radius index of the node.
     * 
     * @returns const unsigned length index of the node
     */
    const unsigned& lengthIndex() const {
        return indices_[1];
    }

    /**
     * Gets the radius index of the node.
     * 
     * @returns const unsigned angle index of the node
     */
    const unsigned& angleIndex() const {
        return indices_[2];
    }

    /**
     * Gets the levels for the node.
     * 
     * @returns const ResLevels levels [length, angle] for the node 
     */
    const ResLevels& levels() const {
        return levels_;
    }

    /**
     * Gets the indices for the node.
     * 
     * @returns const NodeIndices indices [radius, length, angle]
     */
    const NodeIndices& indices() const {
        return indices_;
    }

    /**
     * Checks if the provided indices have been explored?? adds them to explored list if they were not found.
     * @param indices: indices to check for exploration
     * 
     * @returns bool true if the indices have been explored, false otherwise
     */
    bool explored(const NodeIndices& indices) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (explored_.find(IdxToStr(indices)) != explored_.end()) {
            return true;
        }

        explored_.insert(IdxToStr(indices));
        return false;
    }

    /**
     * Writes the details of the node (levels, indices, rank, parent) to an output stream.
     * @param out: stream to write the node information to
     */
    void print(std::ostream& out=std::cout) const {
        out << "length level: " << levels_[0] << ", "
            << "angle level: " << levels_[1] << ", "
            << "radius index: " << indices_[0] << ", "
            << "length index: " << indices_[1] << ", "
            << "angle index: " << indices_[2] << ", "
            << "node rank: " << rank_ << ", "
            << "if parent exists: " << (this->parent() != nullptr) << ".\n ";

        if (this->parent()) {
            out << "Start from: ";
            this->printState(this->parent()->state(), out);
        }

        out << "End at: ";
        this->printState(this->state(), out);
    }

    /**
     * Writes the details of the state (position, quaternion) to an output stream.
     * @param state: state with information to write
     * @param out: stream to write the node information to
     */
    void printState(const State& state, std::ostream& out=std::cout) const {
        auto const& p = state.translation();
        auto const& q = state.rotation();

        out << "[" << p[0] << "," << p[1] << "," << p[2] << ","
            << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "]"
            << std::endl;
    }
};

struct NodeKey {
    template <typename State, typename Traj>
    const State& operator() (const Node<State, Traj>* node) const {
        return node->state();
    }
};

template <typename State>
struct StateNode {
    State state;

    StateNode(const State& s) {
        state = s;
        Eigen::Vector3d tang = (s.rotation().normalized()*Eigen::Vector3d::UnitZ()).normalized();
        state.rotation() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), tang).normalized();
    }
};

struct StateNodeKey {
    template <typename State>
    const State& operator() (const StateNode<State>& n) const {
        return n.state;
    }
};

} // unc::robotics::mpt::impl::prcs

#endif // MPT_IMPL_PRCS_NODE_HPP