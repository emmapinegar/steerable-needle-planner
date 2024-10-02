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
#ifndef SNP_NEEDLE_SPREADING_PRRT_IMPL_H
#define SNP_NEEDLE_SPREADING_PRRT_IMPL_H

#include "prrt_base.hpp"

namespace unc::robotics::mpt::impl::prrt {
template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedleSpreadingPRRT : public
    PlannerBase<NeedleSpreadingPRRT<Scenario, maxThreads, reportStats, NNStrategy>> {
  public:
    using Space = scenario_space_t<Scenario>;
    using State = typename Space::Type;

  private:
    using Planner = NeedleSpreadingPRRT;
    using Base = PlannerBase<Planner>;
    using Distance = typename Space::Distance;
    using Link = scenario_link_t<Scenario>;
    using Traj = link_trajectory_t<Link>;
    using Node = prrt::Node<State, Traj>;
    using Edge = prrt::Edge<State, Traj>;
    using RNG = scenario_rng_t<Scenario, Distance>;
    using Sampler = scenario_sampler_t<Scenario, RNG>;
    using Point = typename Scenario::Position;

    Distance maxDistance_{std::numeric_limits<Distance>::infinity()};
    Distance goalBias_{0.01};

    static constexpr bool concurrent = maxThreads != 1;
    using NNConcurrency = std::conditional_t<concurrent, nigh::Concurrent, nigh::NoThreadSafety>;
    nigh::Nigh<Node*, Space, NodeKey, NNConcurrency, NNStrategy> nn_;

    std::mutex mutex_;
    std::forward_list<Node*> goals_;

    Node* approxRes_{nullptr};
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};

    Atom<std::size_t, concurrent> goalCount_{0};

    ObjectPool<Node, false> startNodes_;

    struct Worker;

    WorkerPool<Worker, maxThreads> workers_;

    /**
     * Records that a goal has been reached with node.
     * @param node: the node reaching the goal
     */
    void foundGoal(Node* node) {
        if constexpr (reportStats) {
            MPT_LOG(INFO) << "found solution with cost " << node->cost() << " angle total " << node->ang_total();
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            goals_.push_front(node);
        }

        ++goalCount_;
    }

    /**
     * Records that a goal has almost been reached with node.
     * @param node: the node that approximately reached the goal
     * @param dist: the distance between the node state and the goal state
     */
    void foundApproxGoal(Node* node, Distance* dist) {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (*dist < bestDist_) {
                MPT_LOG(INFO) << "update approximate solution with dist " << *dist;
                bestDist_ = *dist;
                approxRes_ = node;
            }
            else if (*dist > bestDist_) {
                *dist = bestDist_;
            }
        }
    }

  public:
    template <typename RNGSeed = RandomDeviceSeed<>>
    explicit NeedleSpreadingPRRT(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
        : nn_(scenario.space())
        , workers_(scenario, seed) {
        MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
        MPT_LOG(TRACE) << "Using concurrency: " << log::type_name<NNConcurrency>();
        MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();
    }

    /**
     * Sets the goal sampling bias (if 0 <= bias <= 1).
     * @param bias: new sampling bias
     */
    void setGoalBias(Distance bias) {
        assert(0 <= bias && bias <= 1);
        goalBias_ = bias;
    }

    /**
     * Gets the goal sampling bias.
     * 
     * @returns Distance the current goal bias
     */
    Distance getGoalBias() const {
        return goalBias_;
    }

    /**
     * Sets the maximum distance range for the problem.
     * @param range: new maximum distance range
     */
    void setRange(Distance range) {
        assert(range > 0);
        maxDistance_ = range;
    }

    /**
     * Gets the maximum distance range for the problem. 
     * 
     * @returns Distance the current maximum distance range
     */
    Distance getRange() const {
        return maxDistance_;
    }

    /**
     * Calculates the number of nodes created between all of the workers.
     * 
     * @returns size_t the number of nodes created
     */
    std::size_t size() const {
        return nn_.size();
    }

    /**
     * Adds a starting node to the queue.
     * @param args:
     * 
     */
    template <typename ... Args>
    void addStart(Args&& ... args) {
        std::lock_guard<std::mutex> lock(mutex_);
        Node* node = startNodes_.allocate(Traj{}, nullptr, std::forward<Args>(args)...);
        nn_.insert(node);
    }

    using Base::solveFor;
    using Base::solveUntil;

    /**
     * Starts solving the problem by starting the workers.
     * @param doneFn: function that determines when the worker is done
     * 
     * @returns 
     */
    template <typename DoneFn>
    std::enable_if_t<std::is_same_v<bool, std::result_of_t<DoneFn()>>>
    solve(DoneFn doneFn) {
        if (size() == 0) {
            throw std::runtime_error("there are no valid initial states");
        }

        workers_.solve(*this, doneFn);
    }

    /**
     * Unknown action
     * 
     * @returns true if the problem has been solved, false otherwise
     */
    bool solved() const {
        return goalCount_.load(std::memory_order_relaxed);
    }

    /**
     * Unknown action
     * 
     * @returns bool true if the problem has been approximately solved, false otherwise
     */
    bool approxSolved() const {
        return (approxRes_ != nullptr);
    }

    /**
     * Gets the number of plans found.
     * 
     * @returns size_t the number of plans found 
     */
    std::size_t numPlansFound() const {
        return goalCount_.load(std::memory_order_relaxed);
    }

  private:
    /**
     * Calculates the cost and number of nodes of the path from the node to the root of the tree.
     * @param n: the node to get the cost of 
     * 
     * @returns Distance the cost from the start to the node, size_t the number of nodes in the path
     */
    std::pair<Distance, std::size_t> pathCost(const Node* n) const {
        Distance cost = 0;
        std::size_t size = 0;

        if (n) {
            cost += workers_[0].scenario().FinalStateCost(n->state());
            ++size;

            for (const Node *p ; (p = n->parent()) != nullptr ; n = p) {
                cost += workers_[0].scenario().CurveCost(p->state(), n->state());
                ++size;
            }
        }

        return {cost, size};
    }

    /**
     * Finds he best cost to get to the goal or the approximate cost if the goal has not been reached yet. 
     * 
     * @returns Distance the cost of the best solution (or approximate), size_t the number of nodes in the solution path, Node the goal with the best cost
     */
    std::tuple<Distance, std::size_t, const Node*> bestSolution() const {
        Distance bestCost = std::numeric_limits<Distance>::infinity();
        std::size_t bestSize = 0;
        const Node* bestGoal = nullptr;

        if (goals_.empty() && approxRes_) {
            const Node* goal = approxRes_;
            auto [cost, size] = pathCost(goal);
            bestCost = cost;
            bestSize = size;
            bestGoal = goal;
            return {bestCost, bestSize, bestGoal};
        }

        for (const Node* goal : goals_) {
            auto [cost, size] = pathCost(goal);

            if (cost < bestCost || bestGoal == nullptr) {
                bestCost = cost;
                bestSize = size;
                bestGoal = goal;
            }
        }

        return {bestCost, bestSize, bestGoal};
    }

    /**
     * Links the solution from node back to root.
     * @param node: node to use to link the solution back to root
     * @param fn: function to link the solution
     * 
     * @returns trajectory callback
     */
    template <typename Fn>
    std::enable_if_t< is_trajectory_callback_v< Fn, State, Traj> >
    solutionRecur(const Node* node, Fn& fn) const {
        if (const Node* p = node->parent()) {
            solutionRecur(p, fn);
            fn(p->state(), node->edge().link(), node->state(), true);
        }
    }

    /**
     * Links the solution from node back to root.
     * @param node: node to use to link the solution back to root
     * @param fn: function to link the solution
     * 
     * @returns trajectory reference callback
     */
    template <typename Fn>
    std::enable_if_t< is_trajectory_reference_callback_v< Fn, State, Traj > >
    solutionRecur(const Node* node, Fn& fn) const {
        if (const Node* p = node->parent()) {
            solutionRecur(p, fn);
            fn(p->state(), *node->edge().link(), node->state(), true);
        }
    }

    /**
     * Links the solution from node back to root.
     * @param node: node to use to link the solution back to root
     * @param fn: function to link the solution
     * 
     * @returns waypoint callback
     */
    template <typename Fn>
    std::enable_if_t< is_waypoint_callback_v< Fn, State, Traj > >
    solutionRecur(const Node* node, Fn& fn) const {
        if (const Node* p = node->parent()) {
            solutionRecur(p, fn);
        }

        fn(node->state());
    }

  public:
    /**
     * Gets the path of states that lead to the best solution.
     * 
     * @returns vector<State> the states composing the solution
     */
    std::vector<State> solution() const {
        auto [cost, size, n] = bestSolution();
        std::vector<State> path;

        if (n) {
            path.reserve(size);

            do {
                path.push_back(n->state());
            }
            while ((n = n->parent()) != nullptr);

            std::reverse(path.begin(), path.end());
        }

        return path;
    }

    /**
     * Gets the paths of states for all solutions.
     * 
     * @returns vector<vector<State>> the vectors of state for all the paths to goal
     */
    std::vector<std::vector<State>> allSolutions () const {
        std::vector<std::vector<State>> paths;

        for (const Node* n : goals_) {
            std::vector<State> path;

            if (n) {
                auto [cost, size] = pathCost(n);
                path.reserve(size);

                do {
                    path.push_back(n->state());
                }
                while ((n = n->parent()) != nullptr);

                std::reverse(path.begin(), path.end());
            }

            paths.push_back(path);
        }

        return paths;
    }

    /**
     * Gets the solution for the best solution. 
     * @param fn: function to link the solution
     */
    template <typename Fn>
    void solution(Fn fn) const {
        auto [cost, size, goal] = bestSolution();

        if (goal) {
            solutionRecur(goal, fn);
        }
    }

    /**
     * Prints the number of nodes in graph, number of solutions, best cost, and number of waypoints. 
     */
    void printStats() const {
        MPT_LOG(INFO) << "nodes in graph: " << nn_.size();
        auto [cost, size, goal] = bestSolution();
        MPT_LOG(INFO) << "solutions: " << goalCount_.load() << ", best cost=" << cost
                      << " over " << size << " waypoints";

        if constexpr (reportStats) {
            WorkerStats<true> stats;

            for (unsigned i=0 ; i<workers_.size() ; ++i) {
                stats += workers_[i];
            }

            stats.print();
        }
    }

    /**
     * Gets the cost of the best solution. 
     * 
     * @returns Distance the cost of the best solution (or approximate if the goal has not been reached)
     */
    Distance cost() const {
        auto [cost, size, n] = bestSolution();
        return cost;
    }

    /**
     * Gets the costs of all solutions.
     * 
     * @returns vector<Distance> the costs of each of the paths to the goal
     */
    std::vector<Distance> allCosts() const {
        std::vector<Distance> costs;

        for (const Node* n : goals_) {
            auto [cost, size] = pathCost(n);
            costs.push_back(cost);
        }

        return costs;
    }

  private:
    /**
     * Visits all of the nodes with the visitor. 
     * @param visitor: visitor worker 
     * @param nodes: nodes for the worker to visit
     */
    template <typename Visitor, typename Nodes>
    void visitNodes(Visitor&& visitor, const Nodes& nodes) const {
        for (const Node& n : nodes) {
            visitor.vertex(n.state());

            if (n.parent()) {
                visitor.edge(n.parent()->state());
            }
        }
    }

  public:
    /**
     * Visits the nodes in the graph using workers.
     * @param visitor: visitor worker 
     */
    template <typename Visitor>
    void visitGraph(Visitor&& visitor) const {
        visitNodes(std::forward<Visitor>(visitor), startNodes_);

        for (const Worker& w : workers_) {
            visitNodes(std::forward<Visitor>(visitor), w.nodes());
        }
    }
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedleSpreadingPRRT<Scenario, maxThreads, reportStats, NNStrategy>::Worker
    : public WorkerStats<reportStats> {
    using Stats = WorkerStats<reportStats>;
    using CSampler = typename Scenario::CSampler;
    using Propagator = typename Scenario::Propagator;

    unsigned no_;
    Scenario scenario_;
    RNG rng_;
    std::uniform_real_distribution<Distance> uniform01_;

    ObjectPool<Node> nodePool_;

    CSampler csampler_;
    Propagator propagator_;
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};

    State start_;

  public:
    Worker(Worker&& other)
        : no_(other.no_)
        , scenario_(other.scenario_)
        , rng_(other.rng_)
        , nodePool_(std::move(other.nodePool_))
        , csampler_(scenario_.Config(), scenario_.StartState())
        , propagator_(scenario_.Config()) {

    }

    template <typename RNGSeed>
    Worker(unsigned no, const Scenario& scenario, const RNGSeed& seed)
        : no_(no)
        , scenario_(scenario)
        , rng_(seed)
        , csampler_(scenario.Config(), scenario.StartState())
        , propagator_(scenario_.Config()) {
    }

    /**
     * Planning space for the problem.
     * 
     * @returns planning space for the problem
     */
    decltype(auto) space() const {
        return scenario_.space();
    }

    /**
     * Planning scenario being used. 
     * 
     * @returns planning scenario for the problem
     */
    decltype(auto) scenario() const {
        return scenario_;
    }

    /**
     * Nodes that have been added to the closed set.
     * 
     * @returns the node pool 
     */
    const auto& nodes() const {
        return nodePool_;
    }

    /**
     * Solves the motion planning problem.
     * @param planner: planner for the problem 
     * @param done: the function that determines when the planner is done
     */
    template <typename DoneFn>
    void solve(Planner& planner, DoneFn done) {
        MPT_LOG(TRACE) << "worker running";

        if constexpr (scenario_has_goal_sampler_v<Scenario, RNG>) {
            if (no_ == 0 && planner.goalBias_ > 0) {
                scenario_goal_sampler_t<Scenario, RNG> goalSampler(scenario_);
                Distance scaledBias = planner.goalBias_ * planner.workers_.size();

                MPT_LOG(TRACE) << "using scaled goal bias of " << scaledBias;

                while (!done()) {
                    Stats::countIteration();

                    if (planner.goalCount_.load(std::memory_order_relaxed) >= 1) {
                        goto unbiasedSamplingLoop;
                    }

                    if (uniform01_(rng_) < scaledBias) {
                        Stats::countBiasedSample();
                        addSample(planner, goalSampler(rng_));
                    }
                    else {
                        addSample(planner, csampler_(rng_));
                    }
                }

                return;
            }
        }

unbiasedSamplingLoop:

        while (!done()) {
            Stats::countIteration();
            addSample(planner, csampler_(rng_));
        }

        MPT_LOG(TRACE) << "worker done";
    }

    /**
     * Attempts to add sample to motion plan.
     * @param planner: planner for the problem
     * @param sample: sampled state to try to add
     */
    void addSample(Planner& planner, std::optional<State>&& sample) {
        if (sample) {
            addSample(planner, *sample);
        }
    }

    /**
     * Finds the node with the state closest to the provided state. 
     * @param planner: planner for the problem
     * @param state: state to search for nodes near
     * 
     * @returns Node the nearest node to the state
     */ 
    decltype(auto) nearest(Planner& planner, const State& state) {
        Timer timer(Stats::nearest());
        return planner.nn_.nearest(state);
    }

    /**
     * Attempts to add sample motion.
     * @param planner: planner for the problem 
     * @param randState: random state to try to add
     */
    void addSample(Planner& planner, State& randState) {
        if (scenario_.collision(randState)) {
            return;
        }

        auto [nearNode, d] = nearest(planner, randState).value();

        State newState = randState;

        if (scenario_.PositionDist(nearNode->state(), randState) < snp::EPS) {
            return;
        }

        if (uniform01_(rng_) < scenario_.Config()->start_connect_ratio) {
            auto startState = scenario_.DirectConnectingStart(randState);

            if (startState) {
                planner.addStart(*startState);
            }
        }

        auto propagated = propagator_(nearNode->state(), randState, rng_);

        if (!propagated) {
            return;
        }

        newState = *propagated;

        auto const& newLength = nearNode->length() + snp::CurveLength(nearNode->state(), newState);
        auto const& newAngle  = nearNode->ang_total() + DirectionDifference(nearNode->state().rotation(), newState.rotation());

        // std::cout << "angle total: " << newAngle << std::endl;

        if (!scenario_.valid(newState, newLength, newAngle)) {
            return;
        }

        if (auto traj = validMotion(nearNode->state(), newState)) {
            auto [isGoal, goalDist, goalState] = scenario_goal<Scenario>::check(scenario_, newState);

            Node* newNode = nodePool_.allocate(linkTrajectory(traj), nearNode, newState);
            newNode->length() = newLength;
            newNode->ang_total() = newAngle;
            planner.nn_.insert(newNode);

            if (isGoal) {
                auto const& goalLength = newLength + snp::CurveLength(newState, goalState);
                auto const& goalAngle  = newNode->ang_total() + DirectionDifference(newNode->state().rotation(), goalState.rotation());

                // std::cout << "angle total: " << goalAngle << std::endl;

                if (!scenario_.valid(goalLength)) {
                    return;
                }

                if (snp::IsTheSameState(goalState, newState)) {
                    planner.foundGoal(newNode);
                }
                else {
                    Node* goalNode = nodePool_.allocate(linkTrajectory(traj), newNode, goalState);
                    goalNode->length() = goalLength;
                    goalNode->ang_total() = goalAngle;
                    planner.foundGoal(goalNode);
                }
            }
            else if (!planner.solved() && goalDist < bestDist_) {
                auto const& goalLength = newLength + snp::CurveLength(newState, goalState);
                auto const& goalAngle  = newNode->ang_total() + DirectionDifference(newNode->state().rotation(), goalState.rotation());

                if (!scenario_.valid(goalLength)) {
                    return;
                }
                // std::cout << "angle total: " << goalAngle << std::endl;
                bestDist_ = goalDist;
                planner.foundApproxGoal(newNode, &bestDist_);
            }
        }
    }

    /**
     * Checks if the motion from the provided state to the other state is a valid motion.
     * @param a: starting state
     * @param b: target state
     * 
     * @returns ?? 
     */
    decltype(auto) validMotion(const State& a, const State& b) {
        Timer timer(Stats::validMotion());
        return scenario_.link(a, b);
    }
};

} // namespace unc::robotics::mpt::impl::prrt

#endif // SNP_NEEDLE_SPREADING_PRRT_IMPL_H