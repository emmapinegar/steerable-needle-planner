import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from needle import SteerableNeedle
from environment import ReMINDEnvironment 
import time
import random
from sklearn.neighbors import KDTree


_DEBUG = True
_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'
_UNREACHABLE = 10000

_PATH_COLORS_HEX = ["#021CB1", "#1C3EFF", "#536EFF", "#8697FB"]
_PATH_COLORS_RGB = [[2/255,28/255,177/255], [28/255,62/255,225/255], [83/255,110/255,255/255], [134/255,151/255,251/255]]
_TREE_COLORS_HEX = ["#00A8FF", "#56C5FF", "#87D6FF", "#AAE2FF"]
_TREE_COLORS_RGB = [[0/255,168/255,255/255], [86/255,197/255,255/255], [135/255,214/255,255/255], [170/255,226/255,255/255]]

class TreeNode:
    '''
    Class to hold node state and connectivity for building an RRT
    '''

    def __init__(self, needle_model:SteerableNeedle, parent=None):
        self.needle_model = needle_model
        self.state = needle_model.p
        self.action = needle_model.q
        self.children = []
        self.parent = parent
        self.cost = self.action[0]
        if self.parent is not None:
            self.cost += self.parent.cost

    def add_child(self, child):
        '''
        Add a child node
        '''
        self.children.append(child)


class RRTSearchTree:
    '''
    Searh tree used for building an RRT
    '''
    def __init__(self, root_needle:SteerableNeedle, skull_tree, r_curvature_line, phi_constraint=False, variable_curvature=False):
        '''
        init - initial tree configuration
        '''
        root_needle.phi_constraint = phi_constraint
        root_needle.skull_tree = skull_tree
        root_needle.r_curvature_line = r_curvature_line
        root_needle.variable_curvature = variable_curvature
        self.root = TreeNode(root_needle)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query):
        '''
        Find node in tree closets to s_query
        returns - (nearest node, dist to nearest node)
        '''
        min_d = _UNREACHABLE*1000
        nn = self.root
        for n_i in self.nodes:
            d = get_distance(s_query,n_i)
            l2 = get_state_distance(s_query,n_i.needle_model.p)
            if l2 < 1e-3 and d > 1e-2:
                print(f"min d: {min_d} d: {d} l2: {l2} p: {n_i.needle_model.p} query: {s_query}")
                n_i.needle_model.get_distance(s_query, print_=True)
            if d < min_d and d >= 0.0:
                # print(f"min d: {min_d} d: {d} p: {n_i.needle_model.p} query: {s_query}")
                nn = n_i
                min_d = d
                if min_d <= 1e-12:
                    # print(f"query: {s_query} closest: {nn.needle_model.p} dist: {round(min_d,10)}")
                    return (nn, min_d)

        return (nn, min_d)

    def add_node(self, node, parent):
        '''
        Add a node to the tree
        node - new node to add
        parent - nodes parent, already in the tree
        '''
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)

    def get_states_and_edges(self):
        '''
        Return a list of states and edgs in the tree
        '''
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n):
        '''
        Get the path from the root to a specific node in the tree
        n - node in tree to get path to
        '''
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent
        path.append(n.state)
        path.reverse()
        return path

    def get_back_plan(self, n):
        '''
        Get the path from the root to a specific node in the tree
        n - node in tree to get path to
        '''
        plan = []
        while n.parent is not None:
            plan.append(n.action)
            n = n.parent
        plan.reverse()
        return plan


def get_distance(state, node:TreeNode):
    '''
    Gets the distance between state and Tree node
    '''
    better = True
    if better:
        distance = node.needle_model.get_distance(state)
    else:
        distance = get_state_distance(state, node.state)
    return distance

def get_state_distance(state_a, state_b):
    '''
    Get euclidean distance between the two states
    '''
    magnitude = np.linalg.norm(state_b - state_a)
    return magnitude


class RRT(object):
    '''
    Rapidly-Exploring Random Tree Planner
    '''
    def __init__(self, num_samples, num_dimensions=3, step_length = 1, lims = None,
                 skull_tree=None, r_curvature_line=None, connect_prob = 0.05, collision_func=None, custom_sample_func=None, time_limit=60, variable_curvature=False):
        '''
        Initialize an RRT planning instance
        num_samples (int) - number of samples to try before terminating
        num_dimensions (int) - number of dimensions in state space
        step_length (float) - size of step when extending
        lims (array) - limits for each of the state dimensions
        skull_tree(kdtree) - kdtree used to find closest point in the skull
        r_curvature_line(array) - array storing values that define a line of best fit for radius curvature
        connect_prob (float) - 0-1 goal sampling probability
        collision_func (function) - used to perform collision checking during expansion
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.skull_tree = skull_tree
        self.r_curvature_line = r_curvature_line
        self.connect_prob = connect_prob
        self.in_collision = collision_func
        self.custom_sample_func = custom_sample_func
        self.variable_curvature = variable_curvature
        
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in range(num_dimensions):
                self.limits.append([1,10])
            self.limits = np.array(self.limits)

        self.ranges = np.abs(self.limits[:,1] - self.limits[:,0])
        self.found_path = False
        self.time_limit = time_limit

    def rrt_setup(self, robot_init:SteerableNeedle, goal, phi_constraint):
        self.goal = np.array(goal)
        self.init = np.array(robot_init.p)
        self.start = robot_init
        self.found_path = False
        self.T = RRTSearchTree(robot_init, self.skull_tree, self.r_curvature_line, phi_constraint=phi_constraint, variable_curvature=self.variable_curvature)


    def build_rrt(self, robot_init:SteerableNeedle, goal, phi_constraint=False):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        start_time = time.time()
        self.rrt_setup(robot_init, goal, phi_constraint)

        if self.in_collision(self.init):
            print("start in collision")
            return None, None, _UNREACHABLE, _UNREACHABLE
        elif self.in_collision(self.goal):
            print("goal in collision")
            return None, None, _UNREACHABLE, _UNREACHABLE

        random_state = self.goal
        # Sample and extend
        for k in range(self.K):
            if k%(self.K//10) == 0:
                if _DEBUG:
                    print(f"sample {k}/{self.K}... nodes: {self.T.nodes.__len__()}")
            if time.time() - start_time > self.time_limit:
                break
            (status, new_node) = self.extend(self.T, random_state)
            # print(f"SAMPLE: {random_state}, NEW NODE: {new_node}")
            if status is not _TRAPPED:
                # if _DEBUG:
                #     print("node is not trapped")
                goal_mag = get_state_distance(self.goal, new_node.state)
                if goal_mag == 0.0:
                    print(f"samples: {k}")
                    return self.T.get_back_path(new_node), self.T.get_back_plan(new_node), new_node.needle_model.phi, new_node.needle_model.l
                elif goal_mag <= 5*self.epsilon:
                    if _DEBUG:
                        print("near goal! attempting to extend towards goal")
                    (status, new_node) = self.extend(self.T, self.goal)
                    if status == _REACHED:
                        print(f"samples: {k}")
                        return self.T.get_back_path(new_node), self.T.get_back_plan(new_node), new_node.needle_model.phi, new_node.needle_model.l
            random_state = self.sample()
        return None, None, _UNREACHABLE, _UNREACHABLE
    

    def build_rrt_connect(self, robot_init:SteerableNeedle, goal, phi_constraint=False):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        start_time = time.time()
        self.rrt_setup(robot_init, goal, phi_constraint)

        if self.in_collision(self.init):
            print("start in collision")
            return None, None, _UNREACHABLE, _UNREACHABLE
        elif self.in_collision(self.goal):
            print("goal in collision")
            return None, None, _UNREACHABLE, _UNREACHABLE

        random_state = self.goal

        # Sample and extend
        for k in range(self.K):
            if k%(self.K//10) == 0:
                if _DEBUG:
                    print(f"sample {k}/{self.K}... nodes: {self.T.nodes.__len__()}")
                if time.time() - start_time > self.time_limit:
                    break          
            keep_extending = True
            # print(f"sample: {random_state}")
            while keep_extending:
                (status, new_node) = self.extend(self.T, random_state, rejection=False)
                keep_extending = status == _ADVANCED
                if not status == _TRAPPED:

                    goal_mag = get_state_distance(self.goal, new_node.state)
                    goal_reach_distance = new_node.needle_model.get_distance(self.goal)
                    # print(f"goalmag: {goal_mag} goalreachdist: {goal_reach_distance} sample: {random_state} state: {new_node.state}")
                    # if _DEBUG:
                    # print(f"node is not trapped goal dist : {goal_mag} status: {status} ")
                    if goal_mag <= 5*self.epsilon:
                        (status, new_node) = self.extend(self.T, self.goal, rejection=False)
                        if status == _REACHED:
                            print(f"samples: {k}")
                            return self.T.get_back_path(new_node), self.T.get_back_plan(new_node), new_node.needle_model.phi, new_node.needle_model.l
                    elif goal_reach_distance >= _UNREACHABLE:
                        keep_extending = False

            random_state = self.sample()

        return None, None, _UNREACHABLE, _UNREACHABLE        


    def rebuild_tree(self, robot_init:SteerableNeedle, goal, phi_constraint=True):
        start_time = time.time()
        self.rrt_setup(robot_init, goal, phi_constraint)

        if self.in_collision(self.init):
            print("start in collision")
            return None, None, _UNREACHABLE, _UNREACHABLE
        elif self.in_collision(self.goal):
            print("goal in collision")
            return None, None, _UNREACHABLE, _UNREACHABLE

        # Sample and extend
        for k in range(self.K-1):
            random_state, parent = self.sample(i=k+1)
            if _DEBUG:
                if self.K > 100:
                    if k%(self.K//100) == 0:
                        print(f"sample {k}/{self.K}... nodes: {self.T.nodes.__len__()}")
                else:
                    print(f"sample {k}/{self.K}... nodes: {self.T.nodes.__len__()}")

            if self.K > 10 and k%(self.K//10) == 0 and time.time() - start_time > self.time_limit:
                break          
            keep_extending = True
            while keep_extending:
                (status, new_node) = self.extend(self.T, random_state, parent)
                keep_extending = status == _ADVANCED

                if status == _TRAPPED:

                    print(f"state not reached! {random_state} parent: {parent} k: {k} status: {keep_extending}\n")
                    # break
                    return None, None, _UNREACHABLE, _UNREACHABLE 

        return None, None, _UNREACHABLE, _UNREACHABLE   


    def sample(self, goal=None, i=0):
        '''
        Sample a new configuration
        goal (array) - [x, y, z] goal to sample towards overriding self.goal
        Returns a configuration of size self.n bounded in self.limits
        '''
        # Return goal with connect_prob probability
        
        sample_prob = np.random.rand()
        if sample_prob <= self.connect_prob:
            if goal is None:
                return self.goal
            else:
                return goal
        else:
            if self.custom_sample_func is None:
                new_configuration = np.random.rand(self.n)
                new_configuration = self.ranges*new_configuration
                new_configuration = new_configuration + self.limits[:,0]
            else:
                new_configuration = self.custom_sample_func(i)     
            return new_configuration


    def extend(self, T:RRTSearchTree, sample, parent):
        '''
        Perform rrt extend operation.
        T (RRTSearchTree) - search tree to work on
        sample (array) - [x, y, z] new configuration to extend towards
        rejection (bool) - if true rejects sample if the needle limits are not capable of reaching the sample (ignoring obstacles), default if false
        returns - tuple of (status, TreeNode)
           status can be: _TRAPPED, _ADVANCED or _REACHED
        '''
        (nearest_node, magnitude) = T.find_nearest(parent)
        print(f"\nsample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude}")
        if magnitude > 1e-5:
            print(f"\nsample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude} printing parent")
            nearest_node.parent.needle_model.move_needle(nearest_node.needle_model.p, print_=True)
            return (_TRAPPED, nearest_node)
        nearest_node.needle_model.get_new_lims(sample, print_=True)
        q, phi = nearest_node.needle_model.ik(sample, print_=True)
        if q is not None:
            
            magnitude = q[0]
            p = nearest_node.needle_model.fk(q)
            if q[0] > self.epsilon:
                q = (self.epsilon, q[1], q[2])
            p = nearest_node.needle_model.fk(q)

            new_needle = nearest_node.needle_model
            needles = []
            while magnitude >= self.epsilon:
                if q is not None:
                    
                    if q[0] > self.epsilon:
                        q = (self.epsilon, q[1], q[2])
                    p = new_needle.fk(q)
                    # if q[1] < 1e-5:
                    #     new_needle = new_needle.move_needle(p[0:3,3], q=q, print_=True)
                    # else:
                    new_needle = new_needle.move_needle(p[0:3,3], print_=True)
                    
                    if new_needle is not None:
                        q, phi = new_needle.ik(sample,print_=False)
                        magnitude = new_needle.get_distance(sample)
                        new_needle_node = TreeNode(new_needle)
                    else:
                        print(f"p: {p[0:3,3]} q: {q} needle none")
                        return (_TRAPPED, nearest_node)
                    if not self.in_collision(new_needle_node.state):
                        needles.append(new_needle_node)
                    else: 
                        print(f"\nsample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude}")
                        print("path not added due to collision")
                        return (_TRAPPED, nearest_node)
                else:
                    new_needle.ik(sample, print_=True)
                    nearest_node.needle_model.ik(sample, print_=True)
                    print("q is None")
                    return (_TRAPPED, nearest_node)

            new_needle = nearest_node.needle_model.move_needle(sample)
            new_node = TreeNode(new_needle)
            T.add_node(new_node, nearest_node)
            new_mag = get_state_distance(sample, new_node.state) 
            new_reach_dist = new_node.needle_model.get_distance(sample)     
            if new_mag <= self.epsilon:
                return (_REACHED, new_node)
            else:
                return (_TRAPPED, new_node)
            # return (_REACHED, nearest_node)

        
        print(f"sample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude} ik says not reachable! print parent")
        nearest_node.parent.needle_model.move_needle(nearest_node.needle_model.p, print_=True)
        print("printing sample ik!")
        q, phi = nearest_node.needle_model.ik(sample, print_=True)
        nearest_node.needle_model.get_new_lims(sample, print_=True)
        return (_TRAPPED, None)


    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

