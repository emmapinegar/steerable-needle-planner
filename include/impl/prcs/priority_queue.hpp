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
#ifndef MPT_IMPL_PRCS_PQUEUE_HPP
#define MPT_IMPL_PRCS_PQUEUE_HPP

#include "node.hpp"

#include <queue>
#include <mutex>

namespace unc::robotics::mpt::impl::prcs {

/**
 * PriorityQueue for RCS style planners. 
 */
template <typename State, typename Traj>
class PriorityQueue {
    using Node = prcs::Node<State, Traj>;

    struct cmp {
        bool operator() (const Node* n1, const Node* n2) const {
            return n1->rank() > n2->rank();
        }
    };

    using Queue = std::priority_queue<Node*, std::vector<Node*>, cmp>;
    Queue q_;
    std::mutex mutex_;

  public:
    PriorityQueue() {
    }

    /**
     * Adds node to the priority queue.
     * @param node: node to add to the priority queue
     */
    void push(Node* node) {
        std::lock_guard<std::mutex> lock(mutex_);

        q_.push(node);
    }

    /**
     * Removes the highest priority node from the queue. 
     * 
     * @returns Node highest priority node 
     */
    Node* pop() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (q_.empty()) {
            return nullptr;
        }

        auto node = q_.top();
        q_.pop();

        return node;
    }

    /**
     * Checks if the priority queue is empty.
     * 
     * @returns bool true if the priority queue is empty, false otherwise
     */
    bool empty() const {
        return q_.empty();
    }

    /**
     * Gets the size of the priority queue.
     * 
     * @returns size_t number of nodes in the priorit queue
     */
    std::size_t size() const {
        return q_.size();
    }
};

} // unc::robotics::mpt::impl::prcs

#endif // MPT_IMPL_PRCS_PQUEUE_HPP