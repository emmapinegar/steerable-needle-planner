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

//! @author Jeff Ichnowski

#pragma once
#ifndef MPT_IMPL_PRRT_NODE_HPP
#define MPT_IMPL_PRRT_NODE_HPP

#include "edge.hpp"
#include <utility>

namespace unc::robotics::mpt::impl::prrt {
/**
 * Nodes for the the motion planner to track states and path attributes.
 */
template <typename State, typename Traj>
class Node {
    using Scalar = typename State::Distance;

    State state_;
    Edge<State, Traj> parent_;
    Scalar traj_length_{0};
    Scalar cost_{0};
    Scalar ang_total_{0};

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
    Node(Traj&& traj, Node *parent, Args&& ... args)
        : state_(std::forward<Args>(args)...)
        , parent_(std::move(traj), parent)
    {
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
        return cost_;
    }

    /**
     * Gets the cumulative cost of the trajectory up to this node.
     * 
     * @returns const Scalar trajectory cost
     */
    const Scalar& cost() const {
        return cost_;
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
};

struct NodeKey {
    template <typename State, typename Traj>
    const State& operator() (const Node<State, Traj>* node) const {
        return node->state();
    }
};

} // unc::robotics::mpt::impl::prrt

#endif // MPT_IMPL_PRRT_NODE_HPP