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
#ifndef MPT_IMPL_PRCS_LOOKAHEAD_PQUEUE_HPP
#define MPT_IMPL_PRCS_LOOKAHEAD_PQUEUE_HPP

#include "node.hpp"

#include <queue>
#include <set>
#include <mutex>

namespace unc::robotics::mpt::impl::prcs {

/**
 * LookaheadPriorityQueue for RCS* planners. 
 */
template <typename State, typename Traj, int maxThreads>
class LookaheadPriorityQueue {
    using Node = prcs::Node<State, Traj>;
    using Scalar = typename State::Distance;

    struct costCmp {
        bool operator() (const Node* n1, const Node* n2) const {
            return n1->f() > n2->f();
        }
    };
    using CostQueue = std::priority_queue<Node*, std::vector<Node*>, costCmp>;
    using QueueVec = std::vector<CostQueue>;

    unsigned lookAhead_{0};
    unsigned minActiveRank_{0};
    unsigned maxActiveRank_{0};
    std::pair<unsigned, Scalar> toPop_{};
    std::size_t counter_{0};

    std::set<unsigned> ranksInQueue_;
    QueueVec queueVec_;

    std::mutex mutex_;

  public:
    LookaheadPriorityQueue() {
    }

    /**
     * Sets the number of places to lookahead in the queue.
     * @param lookAhead: number of places to lookahead
     */
    void setLookAhead(const unsigned& lookAhead) {
        lookAhead_ = lookAhead;
    }

    /**
     * Gets the number of places to lookahead in the queue.  
     * 
     * @returns const unsigned value of lookahead
     */
    const unsigned& lookAhead() const {
        return lookAhead_;
    }

    /**
     * Adds node to the priority queue.
     * @param node: node to add to the priority queue
     */
    void push(Node* node) {
        std::lock_guard<std::mutex> lock(mutex_);

        unsigned rank = node->rank();
        if (rank >= queueVec_.size()) {
            queueVec_.resize(rank + 1);
        }
        queueVec_[rank].push(node);
        ranksInQueue_.insert(rank);
        counter_++;
    }

    /**
     * Removes the highest priority node from the queue. 
     * 
     * @returns Node highest priority node 
     */
    Node* pop() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (empty()) {
            return nullptr;
        }

        minActiveRank_ = *(ranksInQueue_.begin());
        maxActiveRank_ = std::min(*(ranksInQueue_.rbegin()), minActiveRank_ + lookAhead_);

        toPop_ = {minActiveRank_, R_INF};
        for (unsigned i = minActiveRank_; i <= maxActiveRank_; ++i) {
            CostQueue& queue = queueVec_[i];

            if (queue.empty()) {
                continue;
            }

            if (queue.top()->f() < toPop_.second) {
                toPop_ = {i, queue.top()->f()};
            }
        }

        CostQueue& queueToPop = queueVec_[toPop_.first];
        if (queueToPop.empty()) {
            throw std::runtime_error("[Lookahead priority queue] the queue to pop is empty!");
        }

        auto node = queueToPop.top();
        queueToPop.pop();

        unsigned rank = node->rank();
        if (queueToPop.empty()) {
            ranksInQueue_.erase(rank);
        }
        counter_--;

        return node;
    }

    /**
     * Checks if the priority queue is empty.
     * 
     * @returns bool true if the priority queue is empty, false otherwise
     */
    bool empty() const {
        return counter_ == 0;
    }

    /**
     * Gets the size of the priority queue.
     * 
     * @returns size_t number of nodes in the priorit queue
     */
    std::size_t size() const {
        return counter_;
    }
};

} // namespace unc::robotics::mpt::impl::prcs

#endif // MPT_IMPL_PRCS_LOOKAHEAD_PQUEUE_HPP