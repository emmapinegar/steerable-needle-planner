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
#ifndef SNP_NEEDLE_PRCS_STAR_IMPL_H
#define SNP_NEEDLE_PRCS_STAR_IMPL_H

#include "prcs_base.hpp"
#include "priority_queue.hpp"
#include "lookahead_priority_queue.hpp"

namespace unc::robotics::mpt::impl::prcs {
template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedlePRCSStar : public PlannerBase<NeedlePRCSStar<Scenario, maxThreads, reportStats, NNStrategy>> {
  public:
    using Space = scenario_space_t<Scenario>;
    using State = typename Space::Type;

  private:
    using Planner = NeedlePRCSStar;
    using Base = PlannerBase<Planner>;
    using Distance = typename Space::Distance;
    using Link = scenario_link_t<Scenario>;
    using Traj = link_trajectory_t<Link>;
    using Node = prcs::Node<State, Traj>;
    using Edge = prcs::Edge<State, Traj>;
    using PriorityQueue = prcs::PriorityQueue<State, Traj>;
    using LookaheadPriorityQueue = prcs::LookaheadPriorityQueue<State, Traj, maxThreads>;
    using RNG = scenario_rng_t<Scenario, Distance>;
    using Sampler = scenario_sampler_t<Scenario, RNG>;
    using Point = typename Scenario::Position;
    using Propagator = typename Scenario::Propagator;

    Distance maxDistance_{std::numeric_limits<Distance>::infinity()};

    static constexpr bool concurrent = maxThreads != 1;
    using NNConcurrency = std::conditional_t<concurrent, nigh::Concurrent, nigh::NoThreadSafety>;

    struct NNNode {
        Node* node;
        State state;

        NNNode(Node* n, const State& s)
            : node(n), state(s) {
        }
    };

    struct NNNodeKey {
        const State& operator() (const NNNode& n) const {
            return n.state;
        }
    };

    nigh::Nigh<NNNode, Space, NNNodeKey, NNConcurrency, NNStrategy> nn_;
    nigh::Nigh<StateNode<State>, Space, StateNodeKey, NNConcurrency, NNStrategy> ic_nn_;
    nigh::Nigh<StateNode<State>, Space, StateNodeKey, NNConcurrency, NNStrategy> ic_invalid_nn_;

    Propagator propagator_;

    LookaheadPriorityQueue queue_;

    std::mutex mutex_;
    std::mutex startMutex_;
    std::mutex terminationMutex_;
    std::mutex activeMutex_;
    std::forward_list<Node*> goals_;
    Distance bestCost_{std::numeric_limits<Distance>::infinity()};
    snp::TimePoint start_time_;
    using ResultSeq = std::vector<std::pair<float, Distance>>;
    ResultSeq resultWithTime_;
    unsigned minValidateRank_{10};

    Node* approxRes_{nullptr};
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};

    Atom<std::size_t, concurrent> goalCount_{0};

    int numActivateWorkers_{0};

    ObjectPool<Node, false> startNodes_;

    struct Worker;

    WorkerPool<Worker, maxThreads> workers_;

    /**
     * Records that a goal has been reached with node.
     * @param node: the node reaching the goal
     */
    void foundGoal(Node* node) {
        if constexpr (reportStats) {
            MPT_LOG(INFO) << "found solution with cost " << node->cost() << " angle total " << node->parent()->ang_total();
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            goals_.push_front(node);
            bestCost_ = std::fmin(bestCost_, node->cost());
            resultWithTime_.push_back({std::chrono::duration_cast<std::chrono::duration<float>>(snp::Clock::now() - start_time_).count(), node->cost()});
        }

        ++goalCount_;

        bestDist_ = 0.0;
    }

    /**
     * Records that a goal has almost been reached with node.
     * @param node: the node that approximately reached the goal
     * @param goalState: the goal state that was approximately reached
     * @param nodePool: 
     * @param dist: the distance between the node state and the goal state
     */
    std::optional<Node*> foundApproxGoal(Node* node, const State& goalState, ObjectPool<Node>& nodePool,
                                         Distance* dist) {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (*dist < bestDist_) {
                MPT_LOG(INFO) << "update approximate solution with dist " << *dist;
                bestDist_ = *dist;
                approxRes_ = nodePool.allocate(linkTrajectory(true), node, goalState);
                return approxRes_;
            }
            else if (*dist > bestDist_) {
                *dist = bestDist_;
            }
        }

        return {};
    }

    /**
     * Locks and activates a worker. 
     */
    void addActivateWorker() {
        std::lock_guard<std::mutex> lock(activeMutex_);
        numActivateWorkers_++;
    }

    /**
     * Locks and removes a worker.
     */
    void removeActivateWorker() {
        std::lock_guard<std::mutex> lock(activeMutex_);
        numActivateWorkers_--;
    }

  public:
    template <typename RNGSeed = RandomDeviceSeed<>>
    explicit NeedlePRCSStar(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
        : nn_(scenario.space())
        , workers_(scenario, seed)
        , propagator_(scenario.Config()) {
        MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
        MPT_LOG(TRACE) << "Using concurrency: " << log::type_name<NNConcurrency>();
        MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();

        MPT_LOG(INFO) << "Using planner: " << "NeedlePRCS*";
        queue_.setLookAhead(scenario.Config()->look_ahead);
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
     * Calculates the number of interations run between all of the workers.
     * 
     * @returns size_t the number of iterations run
     */
    std::size_t iterations() const {
        std::size_t sum = 0;

        for (const Worker& w : workers_) {
            sum += w.numIterations();
        }

        return sum;
    }

    /**
     * Calculates the number of nodes created between all of the workers.
     * 
     * @returns size_t the number of nodes created
     */
    std::size_t size() const {
        std::size_t sumSize = 0;

        for (const Worker& w : workers_) {
            sumSize += w.nodes().size();
        }

        return sumSize;
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
        node->valid() = true;
        node->state().rotation().normalize();
        nn_.insert(NNNode(node, node->state()));
        ic_nn_.insert(StateNode(node->state()));
        node->costToGo() = workers_[0].scenario().validator().CostToGo(node->state());
        queue_.push(node);
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
        if (nn_.size() == 0) {
            throw std::runtime_error("there are no valid initial states");
        }

        start_time_ = snp::Clock::now();
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
     * Checks if the planner has exhausted all options.
     * 
     * @returns bool true if the planner has been exhausted and has no remaining options, false otherwise
     */
    bool exhausted() const {
        return (numActivateWorkers_ == 0) && (queue_.empty());
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

    /**
     * Gets the time to get a result from the planner.
     * 
     * @returns ResultSeq resutls of the planner and time
     */
    const ResultSeq& resultWithTime() const {
        return resultWithTime_;
    }

  private:
    /**
     * Visits all of the nodes with the visitor. 
     * @param visitor: visitor worker 
     * @param nodes: nodes for the worker to visit
     */
    template <typename Visitor, typename Nodes>
    void visitNodes(Visitor&& visitor, const Nodes& nodes) const {
        for (const auto& n : nodes) {
            visitor.vertex(n->state());

            if (n->parent()) {
                visitor.edge(n->parent()->state());
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
        for (const Worker& w : workers_) {
            visitNodes(std::forward<Visitor>(visitor), w.nodes());
        }
    }
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedlePRCSStar<Scenario, maxThreads, reportStats, NNStrategy>::Worker
    : public WorkerStats<reportStats> {
    using Stats = WorkerStats<reportStats>;

    unsigned no_;
    Scenario scenario_;
    RNG rng_;

    ObjectPool<Node> nodePool_;
    std::queue<Node*> bin_;
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};
    Distance configTolerance_{0};
    unsigned initNum_{1};
    std::vector<Distance> radList_;
    std::size_t numIterations_{0};
    std::vector<const Node*> closed_;

    enum RefineType {
        SHORTER=0,
        LONGER,
        LEFT,
        RIGHT
    };

  public:
    Worker(Worker&& other)
        : no_(other.no_)
        , scenario_(other.scenario_)
        , rng_(other.rng_)
        , nodePool_(std::move(other.nodePool_)) {
    }

    template <typename RNGSeed>
    Worker(unsigned no, const Scenario& scenario, const RNGSeed& seed)
        : no_(no)
        , scenario_(scenario)
        , rng_(seed) {
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
     * @returns the closed node set
     */
    const auto& nodes() const {
        return closed_;
    }

    /**
     * Number of iterations the planner has run.
     * 
     * @returns (auto) number of iterations the planner has run
     */
    const auto& numIterations() const {
        return numIterations_;
    }

    /**
     * Solves the motion planning problem.
     * @param planner: planner for the problem 
     * @param done: the function that determines when the planner is done
     */
    template <typename DoneFn>
    void solve(Planner& planner, DoneFn done) {
        MPT_LOG(TRACE) << "worker running";

        configTolerance_ = scenario_.validator().ConfigTolerance();
        initNum_ = planner.propagator_.InitialNumberofOrientations();
        radList_ = planner.propagator_.RadCurvList();

        if (no_ > 0) {
            auto const startQueueSize = no_;
            while (!done() && planner.queue_.size() < startQueueSize) {
                std::lock_guard<std::mutex> lock(planner.startMutex_);
            }
        }

        while (!done()) {
            Node* popped_node;
            {
                std::lock_guard<std::mutex> lock(planner.terminationMutex_);
                popped_node = planner.queue_.pop();

                if (popped_node == nullptr) {
                    if (planner.exhausted()) {
                        MPT_LOG(TRACE) << "planner exhausted";
                        break;
                    }

                    continue;
                }

                planner.addActivateWorker();
            }

            Stats::countIteration();
            ++numIterations_;
            process(planner, popped_node, done);

            planner.removeActivateWorker();
        }

        MPT_LOG(TRACE) << "worker done";
    }

    /**
     * Checks if the node satisfies the termination conditions.
     * @param planner: planner for the problem 
     * @param node: to to try to find the goal
     * 
     * @returns bool true if the planner has found a goal with the node
     */
    bool checkTerminateCondition(Planner& planner, Node* node) {
        auto [isGoal, goalDist, goalStates] = scenario_goal<Scenario>::check(scenario_, node->state());

        if (isGoal) {
            if (goalStates.size() < 2) {
                auto const& goalState = goalStates[0];
                auto const goalLength = node->length() + snp::CurveLength(node->state(), goalState);
                auto const goalAngle  = node->ang_total() + DirectionDifference(node->state().rotation(), goalState.rotation());
                if (scenario_.valid(goalLength)) {
                    auto const goalCost = node->cost() + scenario_.CurveCost(node->state(), goalState)
                                         + scenario_.FinalStateCost(goalState);
                    if (goalCost < planner.bestCost_) {
                        Node* goalNode = nodePool_.allocate(linkTrajectory(true), node, goalState);
                        goalNode->length() = goalLength;
                        goalNode->cost() = goalCost;
                        goalNode->ang_total() = goalAngle;
                        planner.foundGoal(goalNode);
                    }

                    return true;
                }
            }
            else {
                auto const goalLength0 = node->length() + snp::CurveLength(node->state(), goalStates[0]);
                auto const goalLength1 = goalLength0 + snp::CurveLength(goalStates[0], goalStates[1]);

                auto const goalAngle0 = node->ang_total() + DirectionDifference(node->state().rotation(), goalStates[0].rotation());
                auto const goalAngle1 = goalAngle0 + DirectionDifference(goalStates[0].rotation(), goalStates[1].rotation());
                if (scenario_.valid(goalLength1)) {
                    auto const goalCost0 = node->cost() + scenario_.CurveCost(node->state(), goalStates[0]);
                    auto const goalCost1 = goalCost0 + scenario_.CurveCost(goalStates[0], goalStates[1])
                                         + scenario_.FinalStateCost(goalStates[1]);

                    if (goalCost1 < planner.bestCost_) {
                        Node* transNode = nodePool_.allocate(linkTrajectory(true), node, goalStates[0]);
                        transNode->length() = goalLength0;
                        transNode->cost() = goalCost0;
                        transNode->ang_total() = goalAngle0;

                        Node* goalNode = nodePool_.allocate(linkTrajectory(true), transNode, goalStates[1]);
                        goalNode->length() = goalLength1;
                        goalNode->cost() = goalCost1;
                        goalNode->ang_total() = goalAngle1;
                        planner.foundGoal(goalNode);
                    }

                }
            }
        }
        else if (!planner.solved() && goalDist < bestDist_) {
            auto goalState = goalStates[0];
            auto const& goalLength = node->length() + snp::CurveLength(node->state(), goalState);
            auto const goalAngle  = node->ang_total() + DirectionDifference(node->state().rotation(), goalState.rotation());
            if (scenario_.valid(goalLength)) {
                bestDist_ = goalDist;
                auto goalNode = planner.foundApproxGoal(node, goalState, nodePool_, &bestDist_);
                if (goalNode) {
                    (*goalNode)->length() = goalLength;
                    (*goalNode)->cost() = node->cost() + scenario_.CurveCost(node->state(), goalState)
                                          + scenario_.FinalStateCost(goalState);
                    (*goalNode)->ang_total() = goalAngle;
                }
            }
        }

        return false;
    }

    /**
     * Processes the node, validating and refining if applicable.
     * @param planner: planner for the problem
     * @param node: node to process
     * @param done: function to determine if termination conditions are satisfied
     */
    template <typename DoneFn>
    void process(Planner& planner, Node* node, DoneFn done) {
        State from = node->state();
        // MPT_LOG(INFO) << "node state " << from.rotation();
        if (node->parent()) {
            if (planner.bestCost_ < node->parent()->f() + EPS) {
                recycle(node);
                return;
            }

            node->length() = node->parent()->length() + planner.propagator_.Length(node->lengthIndex());
            from = planner.propagator_.ComputeStartPose(node->parent()->state(), node->angleIndex());
            // MPT_LOG(INFO) << "parent ang " << node->parent()->ang_total() << " node ang " << DirectionDifference(node->parent()->state().rotation(), node->state().rotation());
            node->ang_total() = node->parent()->ang_total() + DirectionDifference(node->parent()->state().rotation(), node->state().rotation()); // TODO: this may need to go somewhere else
        }

        const bool inheritValidation = node->valid();
        bool inevitableCollision = similarNode(planner.ic_invalid_nn_, node->state(), configTolerance_);

        if (!inevitableCollision && validNode(planner, node)) {
            if (auto traj = validMotion(planner, node, from)) {
                auto const validResult = checkTerminateCondition(planner, node);

                if (done()) {
                    return;
                }

                if (!validResult && node->parent() && node->rank() >= planner.minValidateRank_
                    && !similarNode(planner.ic_nn_, node->state(), 1.0))
                {
                    if (!scenario_.validReachableSpace(node->state())) {
                        inevitableCollision = true;
                        planner.ic_invalid_nn_.insert(StateNode(node->state()));
                    }
                    else {
                        planner.ic_nn_.insert(StateNode(node->state()));
                    }
                }

                if (!inevitableCollision) {
                    expand(planner, node);
                    closed_.push_back(node);
                }
            }
        }

        if (!node->parent()) {
            return;
        }

        auto shorter = refine(planner, node, SHORTER);

        if (node->valid()) {
            if (shorter) {
                shorter->valid() = true;
            }

            if (!inevitableCollision) {
                auto longer = refine(planner, node, LONGER);
                if (inheritValidation && longer) {
                    longer->valid() = true;
                }
            }
        }

        refine(planner, node, LEFT);
        refine(planner, node, RIGHT);

        if (!node->valid()) {
            recycle(node);
        }
    }
 
    /**
     * Checks if the state is too similar to a reference node before adding.
     * @param planner: planner for the problem
     * @param refNode: potentially similar node
     * @param state: state to compare to node before it's added
     * 
     * @returns bool true if the state is too similar to the reference, false otherwise (node was added)
     */
    bool similarState(Planner& planner, Node* refNode, State state) {
        Timer timer(Stats::nearest());
        std::vector<std::pair<NNNode, Distance>> nbh;
        state.rotation().normalize();
        planner.nn_.nearest(nbh, state, nbh.max_size(), configTolerance_);

        bool skipInsertion = false;
        Distance new_cost = refNode->cost() + EPS;
        for (auto const& n : nbh) {
            if (n.first.node != refNode) {
                if (n.first.node->cost() < new_cost) {
                    return true;
                }
            }
            else {
                skipInsertion = true;
            }
        }

        if (!skipInsertion) {
            planner.nn_.insert(NNNode(refNode, state));
        }

        return false;
    }

    /**
     * Checks if there is a node within a radius of the provided state.
     * @param nn:
     * @param state: state to compare to existing nodes
     * @param rad: radius to use to determine if states are too similar
     * 
     * @returns bool true if there is a node that is too similar to the state, false otherwise
     */
    template<typename NN>
    bool similarNode(NN& nn, State state, const Distance rad) {
        Timer timer(Stats::nearest());
        Vec3 tang = (state.rotation().normalized()*Vec3::UnitZ()).normalized();
        state.rotation() = Quat::FromTwoVectors(Vec3::UnitZ(), tang).normalized();
        auto neig = nn.nearest(state);
        if (neig && neig->second < rad) {
            return true;
        }

        return false;
    }

    /**
     * Checks if the node is valid.
     * @param planner: planner for the problem
     * @param node: node to validate
     * 
     * @returns (auto) bool true if the node is valid, false otherwise
     */
    decltype(auto) validNode(Planner& planner, Node* node) {
        if (!scenario_.valid(node->state(), node->length(), node->ang_total()) || planner.bestCost_ < node->f() + EPS) {
            node->valid() = false;
            return false;
        }
        else if (node->valid()) {
            return true;
        }

        if (node->parent()
            && node->angleIndex() == 0
            && node->radIndex() == node->parent()->radIndex()
            && node->parent()->lengthIndex() > 0)
        {
            return false;
        }


        return true;
    }

    /**
     * Checks if the motion from the provided state to the node is a valid motion.
     * @param planner: planner for the problem 
     * @param node: node to add on top of in the validation process
     * @param from: state to add on top of the given node
     * 
     * @returns (auto) bool true if the motion between the node and the state is valid, false otherwise 
     */
    decltype(auto) validMotion(Planner& planner, Node* node, const State& from) {
        if (node->valid()) {
            return true;
        }

        Timer timer(Stats::validMotion());
        auto const& lengthIdx = node->lengthIndex();
        auto const& baseMotion = planner.propagator_.BaseMotion(node->radIndex(), lengthIdx);

        unsigned offset = 0;
        if (lengthIdx > 0 && lengthIdx % 2 == 0) {
            offset = planner.propagator_.BaseMotion(node->radIndex(), lengthIdx/2).size();
        }

        if (scenario_.validator().ValidMotion(from, baseMotion, offset)) {
            node->valid() = true;
            return true;
        }

        return false;
    }
 
    /**
     * Unknown action.
     * @param planner: planner for the problem
     * @param node: node to block? 
     */
    void block(Planner& planner, Node* node) {
        NodeIndices indices = node->indices();
        for (unsigned i = 0; i < planner.propagator_.MaxLengthIndex(); ++i) {
            indices[1] = i;
            node->parent()->explored(indices);
        }
    }

    /**
     * Expands off of the current node and adds them to the queue for processing.
     * @param planner: planner for the problem 
     * @param node: node to expand from
     */
    void expand(Planner& planner, Node* node) {
        for (auto r_i = 0; r_i < radList_.size(); ++ r_i) {
            if (radList_[r_i] == std::numeric_limits<Distance>::infinity()) {
                addNewNode(planner, node, r_i, 0, 0);
            }
            else {
                for (auto a_idx = 0; a_idx < initNum_; ++a_idx) {
                    addNewNode(planner, node, r_i, 0, 0, 0, a_idx);
                }
            }
        }
    }

    /**
     * Refines the characterisitcs for the given node?
     * @param planner: planner for the problem
     * @param node: node to refine
     * @param type: type of refinement (SHORTER, LONGER, LEFT, RIGHT)
     * 
     * @returns Node resulting from refinement
     */
    Node* refine(Planner& planner, Node* node, const RefineType& type) {
        if (!node->parent()) {
            throw std::runtime_error("[ERROR] Cannot compute finer motion for the root!");
        }

        if (type == SHORTER || type == LONGER) {
            if (node->lengthLevel() == planner.propagator_.MaxLengthLevel()) {
                return nullptr;
            }
        }
        else {
            if (radList_[node->radIndex()] == std::numeric_limits<Distance>::infinity()) {
                return nullptr;
            }

            if (node->angleLevel() == planner.propagator_.MaxAngleLevel()) {
                return nullptr;
            }
        }

        ResLevels newLevels = node->levels();
        NodeIndices newIndices = node->indices();

        switch (type) {
        case SHORTER: {
            if (node->lengthLevel() == 0) {
                newIndices[1] += 1;
            }
            else {
                newIndices[1] = newIndices[1] * 2 + 1;
            }

            newLevels[0]++;
            break;
        }

        case LONGER: {
            if (node->lengthLevel() == 0) {
                return nullptr;
            }

            newIndices[1] *= 2;
            newLevels[0]++;
            break;
        }

        case LEFT: {
            if (node->angleLevel() == 0) {
                newIndices[2] += initNum_;
            }
            else {
                newIndices[2] *= 2;
            }

            newLevels[1]++;
            break;
        }

        case RIGHT: {
            if (node->angleLevel() == 0) {
                return nullptr;
            }

            newIndices[2] = newIndices[2] * 2 + 1;
            newLevels[1]++;
            break;
        }
        }

        if (node->parent()->explored(newIndices)) {
            return nullptr;
        }

        return addNewNode(planner, node->parent(), newIndices[0], newLevels[0], newLevels[1],
                          newIndices[1], newIndices[2]);
    }

    /**
     * Adds the new node to the queue for processing.
     * @param planner:
     * @param parent: parent node for new node
     * @param radIndex: motion primitve radius of curvature index
     * @param lengthLevel: motion primitive length level
     * @param angleLevel: motion primitive angle level
     * @param lengthIndex: motion primitive length index, default is 0
     * @param angleIndex: motion primitve angle index, default is 0
     * 
     * @returns Node the new node that has been added
     */
    Node* addNewNode(Planner& planner, Node* parent, const unsigned& radIndex, const unsigned& lengthLevel,
                     const unsigned& angleLevel, const unsigned& lengthIndex=0, const unsigned& angleIndex=0) {
        const State& pState = parent->state();
        auto from = planner.propagator_.ComputeStartPose(pState, angleIndex);
        auto duplicatedStart = similarState(planner, parent, from);

        if (duplicatedStart) {
            return nullptr;
        }

        auto propagated = planner.propagator_(from, radIndex, lengthIndex);

        if (!propagated) {
            return nullptr;
        }

        Node* node;
        const State& endState = *propagated;
        if (bin_.empty()) {
            node = nodePool_.allocate(linkTrajectory(true), parent, endState);
        }
        else {
            node = bin_.front();
            bin_.pop();
            node->reset(linkTrajectory(true), parent, endState);
        }

        node->setResolution({lengthLevel, angleLevel}, {radIndex, lengthIndex, angleIndex});
        node->cost() = parent->cost() + scenario_.CurveCost(pState, endState);
        node->costToGo() = scenario_.validator().CostToGo(endState);
        node->ang_total() = node->parent()->ang_total() + DirectionDifference(pState.rotation(), endState.rotation());
        planner.queue_.push(node);
        return node;
    }

    /**
     * Recycles the given node.
     * @param node: node to be recycled
     */
    void recycle(Node* node) {
        bin_.push(node);
    }
};

} // namespace unc::robotics::mpt::impl::prcs

#endif // SNP_NEEDLE_PRCS_STAR_IMPL_H