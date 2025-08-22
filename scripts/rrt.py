import numpy as np
import numpy.typing as npt
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
    """
    Class to hold node state and connectivity for building an RRT
    """

    def __init__(self, needle_model:SteerableNeedle, parent:"TreeNode"=None):
        """
        Creates an instance of TreeNode.

        Parameters:
            needle_model (SteerableNeedle): the needle model for the TreeNode
            parent (TreeNode | None): the TreeNode that is a parent of this TreeNode, default is None
        """
        self.needle_model = needle_model
        self.state = needle_model.p
        self.action = needle_model.q
        self.children = []
        self.parent:TreeNode = parent
        self.cost = self.action[0]
        if self.parent is not None:
            self.cost += self.parent.cost

    def add_child(self, child:'TreeNode'):
        """
        Add a child TreeNode associated with this TreeNode.

        Parameters: 
            child (TreeNode): the TreeNode that is a child of this TreeNode
        """
        self.children.append(child)


class RRTSearchTree:
    """
    Searh tree used for building an RRT.
    """

    def __init__(self, root_needle:SteerableNeedle, skull_tree:KDTree, r_curvature_line:npt.NDArray, phi_constraint:bool=False, variable_curvature:bool=False):
        """
        Creates an instance of RRTSearch Tree.

        root_needle (SteerableNeedle): the SteerableNeedle that is the start of the tree
        skull_tree (KDTree): the tree containing the skull points to get the closest skull point to a needle location
        r_curvature_line (ndarray): the coefficents for the line fitted to the torque/curvature data
        phi_constraint (bool): enforces the phi constraint if True, default is False
        variable_curvature (bool): recalculates the needle curvature limits at each point instead of using a fixed value if True, default is False
        """
        root_needle.phi_constraint = phi_constraint
        root_needle.skull_tree = skull_tree
        root_needle.r_curvature_line = r_curvature_line
        root_needle.variable_curvature = variable_curvature
        self.root = TreeNode(root_needle)
        self.nodes = [self.root]
        self.edges = []

    def find_nearest(self, s_query:npt.NDArray) -> tuple[TreeNode, float]:
        """
        Find TreeNode in RRTSearchTree closest to s_query. Returns early if the TreeNode is within 1e-12 of the query point. 

        Parameters:
            s_query (ndarray): the poit to find the nearest point to

        Returns:
            nn,min_d (tuple[TreeNode,float]): the TreeNode nearest to the point, the distance between the query point and the TreeNode
        """
        min_d:float = _UNREACHABLE*1000
        nn = self.root
        for n_i in self.nodes:
            d = get_distance(s_query,n_i)
            # l2 = get_state_distance(s_query,n_i.needle_model.p)
            # if l2 < 1e-3 and d > 1e-2:
            #     print(f"min d: {min_d} d: {d} l2: {l2} p: {n_i.needle_model.p} query: {s_query}")
            #     n_i.needle_model.get_distance(s_query, print_=True)
            if d < min_d and d >= 0.0:
                # print(f"min d: {min_d} d: {d} p: {n_i.needle_model.p} query: {s_query}")
                nn = n_i
                min_d = d
                if min_d <= 1e-12:
                    # print(f"query: {s_query} closest: {nn.needle_model.p} dist: {round(min_d,10)}")
                    return (nn, min_d)

        return (nn, min_d)

    def add_node(self, node:TreeNode, parent:TreeNode):
        """
        Add a TreeNode to this RRTSearchTree.

        Parameters:
            node (TreeNode): the TreeNode to add to the RRTSearchTree
            parent (TreeNode): the TreeNode that is the parent of node, alread in the RRTSearchTree
        """
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)

    def get_states_and_edges(self) -> tuple[npt.NDArray, npt.NDArray]:
        """
        Return a list of states and edgs in the tree.

        Returns:
            (states,self.edges) (tuple[ndarray,list]): the states and edges between states in this RRTSearchTree 
        """
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def get_back_path(self, n:TreeNode) -> list:
        """
        Get the path from the root to a specific node in the tree.

        Parameters:
            n (TreeNode): starting point for tracing the path back to the root

        Returns:
            path (list): list of states starting at the root, traversing this RRTSearchTree to n
        """
        path = []
        while n.parent is not None:
            path.append(n.state)
            n = n.parent
        path.append(n.state)
        path.reverse()
        return path

    def get_back_plan(self, n:TreeNode) -> list:
        """
        Get the path from the root to a specific node in the tree.

        Parameters:
            n (TreeNode): starting point for tracing the plan back to the root

        Returns:
            plan (list): list of actions starting at the root, traversing this RRTSearchTree to n
        """
        plan = []
        while n.parent is not None:
            plan.append(n.action)
            n = n.parent
        plan.reverse()
        return plan


def get_distance(state, node:TreeNode) -> float:
    """
    Gets the distance between state and TreeNode.

    Parameters:
        state (ndarray): state 
        node (TreeNode): TreeNode to get the distance to state 

    Returns: 
        distance (float): distance between state and node
    """
    better = True
    if better:
        distance = node.needle_model.get_distance(state)
    else:
        distance = get_state_distance(state, node.state)
    return distance

def get_state_distance(state_a:npt.NDArray, state_b:npt.NDArray) -> float:
    """
    Gets the Euclidean distance between the two states.

    Parameters:
        state_a (NDArray): first state for distance calculations
        state_b (NDArray): second state for distance calculations

    Returns:
        magnitude (float): ||state_b - state_a||
    """
    magnitude = np.linalg.norm(state_b - state_a).astype(float)
    return magnitude


class RRT(object):
    """
    Rapidly-Exploring Random Tree Planner
    """

    def __init__(self, num_samples:int, num_dimensions:int=3, step_length:int=1, lims:npt.NDArray= None,
                 skull_tree:KDTree=None, r_curvature_line:npt.NDArray=None, connect_prob:float= 0.05, collision_func=None, custom_sample_func=None, time_limit:float=60, variable_curvature:bool=False):
        """
        Creates an instance of RRT.

        Parameters:
            num_samples (int): maximum number of samples to take
            num_dimensions (int): number of dimensions for state, defaults is 3
            step_length (float): length of step between states, default is 1
            lims ( | None): limits for each of the state dimensions, default is None
            skull_tree (KDTree | None): contains skull points to query to find the closest point to the skull, default is None
            r_curvature_line (NDArray | None): coefficients of line fit to torque/curvature values, default is None
            connect_prob (float): 0-1 probability of sampling goal, default is 0.05
            collision_func (function | None): function to determine if a state is in collision with obstacles, default is None
            custom_sample_func (function | None): function to get a new sample to try to expand to, default is None
            time_limit (float): soft max amount of time to spend searching for a path to the goal
            variable_curvature (bool): recalculates the needle curvature limits at each point instead of using a fixed value if True, default is False
        """
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

    def rrt_setup(self, robot_init:SteerableNeedle, goal:npt.NDArray, phi_constraint:bool):
        """
        Sets up RRT to be ready to search, resets it has been used previously.

        Parameters:
            robot_init (SteerableNeedle): the starting point for searching for a path to the goal
            goal (array): goal position to try to reach
            phi_constraint (bool): enforces the phi constraint if True
        """
        self.goal = np.array(goal)
        self.init = np.array(robot_init.p)
        self.start = robot_init
        self.found_path = False
        self.T = RRTSearchTree(robot_init, self.skull_tree, self.r_curvature_line, phi_constraint=phi_constraint, variable_curvature=self.variable_curvature)


    def rebuild_tree(self, robot_init:SteerableNeedle, goal:npt.NDArray, phi_constraint:bool=True):
        """
        Rebuilds/retraces the searching process and verifies if the samples are reachable from their chosen parents similar to RRT Connect.

        Parameters:
            robot_init (SteerableNeedle): the initial state to start the searching/verification process
            goal (array): goal location to search toward
            phi_constraint (bool): enforces the phi constraint if True, default is False

        Returns:

        """
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
                if self.K > 10:
                    if k%(self.K//10) == 0:
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


    def sample(self, i:int=0):
        """
        Gets a new sample using the custom sample function.

        Parameters:
            i (int): index of the sample to get from the custom function, default is 0

        Returns:
            sample (ndarray): new sample to try to extend towards
        """
        new_configuration = self.custom_sample_func(i)     
        return new_configuration


    def extend(self, T:RRTSearchTree, sample:npt.NDArray, parent:npt.NDArray):
        """
        Tries to extend to the new sample from the defined parent.

        Parameters:
            T (RRTSearchTree): the tree that is being built
            sample (array): the new sample point to try to reach
            parent (array): the location to try to extend to the sample from
        
        Returns:
            (status,node) (tuple[str,TreeNode]): the status of the extension and the TreeNode resulting if the extension was successful (status=_REACHED)
        """
        (nearest_node, magnitude) = T.find_nearest(parent)
        # print(f"\nsample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude}")
        if magnitude > 1e-5:
            # print(f"\nsample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude} printing parent")
            nearest_node.parent.needle_model.move_needle(nearest_node.needle_model.p, print_=False)
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
                    new_needle = new_needle.move_needle(p[0:3,3], print_=True)
                    
                    if new_needle is not None:
                        q, phi = new_needle.ik(sample,print_=False)
                        magnitude = new_needle.get_distance(sample)
                        new_needle_node = TreeNode(new_needle)
                    else:
                        # print(f"p: {p[0:3,3]} q: {q} needle none")
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
                    # print("q is None")
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

        
        print(f"sample: {sample} parent: {parent} nearest: {nearest_node.needle_model.p} dist: {magnitude} ik says not reachable! print parent")
        next_node = nearest_node
        while next_node.parent is not None:
            print()
            next_node.parent.needle_model.move_needle(next_node.needle_model.p, print_=True)
            next_node = next_node.parent

        print("printing sample ik!")
        q, phi = nearest_node.needle_model.ik(sample, print_=True)
        nearest_node.needle_model.get_new_lims(sample, print_=True)
        return (_TRAPPED, None)


    def fake_in_collision(self, q) -> bool:
        """
        Fake function where there are no collisions!

        Parameters:
            q (): 

        Returns:
            collision (bool): always False
        """
        return False

