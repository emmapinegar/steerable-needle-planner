import numpy as np
import matplotlib.pyplot as plt
import json

from sklearn.neighbors import KDTree
from needle import SteerableNeedle

# try:
#     import open3d as o3d
#     import open3d.visualization.gui as gui
# except Exception as e:
#     print("no open3D installed, you can't use the MD envs without it!")
#     # raise Warning("You must install the Open3D package and an OpenCV visualization software to use the MD Environment!")

_DEBUG = False
_BOUNDS = 'Bounds:'
_GOAL = 'Goal:'
_START = 'Start:'
_NEEDLE = 'NeedleRobot:'
_OBSTACLES = 'Obstacles:'
_TRANSFORM = "Transform:"
_SKULL = "Skull:"
_TORQUE = "Torque:"
_PAIRS = "Pairs:"
_MINK = "MinK:"
_OCCUPIED = 1






class ReMINDEnvironment:
    """
    A class to handle the concerns of the planning environments from the ReMIND dataset.
    """

    def __init__(self):
        self.voxel_grid = None
        self.robot = None
        self.goal = [0, 0, 0]
        self.start = [0, 0, 0]
        self.gw = np.eye(4)
        self.transform = np.eye(4)
        self.sphere_center = None
        self.skulltree = None
        self.torque = None
        self.mink = None
        self.pairs = None
        self.line_parser = {_BOUNDS: self.parse_bounds,
                            _TRANSFORM: self.parse_transform,
                            _START: self.parse_start,
                            _GOAL: self.parse_goal,
                            _OBSTACLES: self.parse_obstacles,
                            _SKULL: self.parse_skull,
                            _TORQUE: self.parse_torque,
                            _NEEDLE: self.parse_needle,
                            _PAIRS: self.parse_pairs,
                            _MINK: self.parse_mink
                            }

    
    def read_env(self, env_file_path, variable_curvature):
        """
        Reads in the information about the environment when the file conforms to the following format:
        <typename>: values
        Bounds: x_min x_max y_min y_max z_min z_max
        SphereBounds: x_center y_center z_center radius
        Start: T_xx T_yx T_zx T_dx T_xy T_yy T_zy T_dy T_xz T_yz T_zz T_dz 0 0 0 1
        Goal: x_goal y_goal z_goal
        Obstacles: obstacle_file_name.npy
        Segmentations: segmentation_file_name.npy
        NeedleRobot: el_min el_max k_min k_max phi_min phi_max
        Transform: T_xx T_yx T_zx T_dx T_xy T_yy T_zy T_dy T_xz T_yz T_zz T_dz 0 0 0 1
        CenterSlice: x_slice y_slice z_slice

        """
        env_file = open(env_file_path, 'r')
        file_infos = env_file.readlines()
        self.read_lines(file_infos, variable_curvature)
        # self.change_pair()
        


    def read_lines(self, lines, variable_curvature):
        """

        """
        for l in lines:
            line_info = l.strip().split()
            if line_info[0].startswith('#'):
                continue
            elif not variable_curvature and (line_info[0] == _SKULL or line_info[0] == _TORQUE):
                continue
            try:
                self.line_parser[line_info[0]](line_info[1:])
            except:
                print(f"a line could not be processed! {line_info[0]}")

        self.transforminv = np.linalg.inv(self.transform)
        if variable_curvature:
            self.robot = SteerableNeedle(needle_lims=self.needle_lims, p=self.start, gw=self.gw, skull_tree=self.skulltree, r_curvature_line=self.torque, variable_curvature=variable_curvature)
        else:
            self.robot = SteerableNeedle(needle_lims=self.needle_lims, p=self.start, gw=self.gw)


    def parse_bounds(self, line_data):
        """
        Parse environment bounds.
        """
        self.x_min = float(line_data[0])
        self.x_max = float(line_data[1])
        self.y_min = float(line_data[2])
        self.y_max = float(line_data[3])
        self.z_min = float(line_data[4])
        self.z_max = float(line_data[5])
        self.lims = np.array([[self.x_min, self.x_max], [self.y_min, self.y_max], [self.z_min, self.z_max]])


    def parse_skull(self, line_data):
        """
        Parse sphere bounds of environment.
        """
        skullpoints = np.loadtxt(line_data[0], skiprows=5)
        skullpoints = np.transpose(skullpoints)
        skullpoints = np.concatenate((skullpoints, np.ones((1,np.shape(skullpoints)[1]))))
        skullpoints = np.matmul(self.transform, skullpoints)
        skullpoints = np.transpose(skullpoints[0:3,:])
        self.skulltree = KDTree(skullpoints)



    def parse_obstacles(self, line_data):
        """
        Parse file containing obstacle data.
        """
        obstacle_file = str(line_data[0])
        obstacles_np = np.load(obstacle_file)
        self.voxel_grid = obstacles_np


    def parse_goal(self, line_data):
        """
        Parse goal location.
        """
        if self.goal is not None:
            self.change_goal([float(l) for l in line_data])
            
    

    def parse_start(self, line_data):
        """
        Parse start location.
        """
        if self.start is not None:
            self.gw = np.array([float(l) for l in line_data]).reshape(-1,4)
            self.gw = np.matmul(self.transform, self.gw)
            self.gw[0:3,0] = self.gw[0:3,0]/np.linalg.norm(self.gw[0:3,0])
            self.gw[0:3,1] = self.gw[0:3,1]/np.linalg.norm(self.gw[0:3,1])
            self.gw[0:3,2] = self.gw[0:3,2]/np.linalg.norm(self.gw[0:3,2])
            self.start = self.gw[0:3, 3].reshape(3,)
            if self.robot is not None:
                self.robot.change_gw(self.gw)


    def parse_needle(self, line_data):
        """
        Parse parameters for steerable needle robot.
        """
        self.needle_lims = np.array([float(l) for l in line_data]).reshape(-1,2)


    def parse_transform(self, line_data):
        """
        Parse transformation matrix that converts from pixel indices to world coordinates.
        """
        self.transform = np.array([float(l) for l in line_data]).reshape(-1,4)


    def parse_torque(self, line_data):
        """
        Parse the indices of the center slice of the environment.
        """
        self.torque = np.array([float(line_data[0]), float(line_data[1])])
        self.torque_file = line_data[2]


    def parse_pairs(self, line_data):
        """
        
        """
        self.pairs = np.loadtxt(line_data[0])



    def parse_mink(self, line_data):
        """
        
        """
        self.mink = float(line_data[0])


    def change_pair(self, i=0):
        """
        Changes the start goal pair to a new non trivial combination.
        """
        if self.pairs is not None:
            
            if i >= np.shape(self.pairs)[0]:
                i = np.random.randint(np.shape(self.pairs)[0])
            pair = self.pairs[i, :]
            self.change_start(pair[0:3])
            self.change_goal(pair[3:6])
            print(f"start: {self.start} goal: {self.goal}")

    def change_start(self, start, convert_world=True):
        """
        Changes starting point for the needle.

        Parameters:
        start (3x1): new starting point for needle

        """
        if convert_world:
            start = self.convert_to_world(start).reshape(3,)
        else:
            start = start.reshape(3,)
        self.robot.change_p(start)
        self.start = self.robot.p
        print(f"start changed: {self.start}")
        print(f"gw: {self.robot.gw}")


    def change_gw(self, gw):
        """
        Changes starting transformation matrix for the needle.

        Parameters:
        gw (4x4): new transformation matrix for needle

        """
        self.robot.change_gw(gw)
        self.start = self.robot.p


    def change_goal(self, goal):
        """
        Changes the goal to the provided goal.
        """
        self.goal = self.convert_to_world(goal).reshape(3,)
        print(f"goal: {goal} converted: {self.goal}")

    def sample(self):
        """
        Sample a coordinate within the limits of the rectangular or spherical bounds of the environment.

        Returns:
        samp (3x1): sample generated
        """
        if self.sphere_center is not None:
            samp = np.random.rand(3,)
            phi = samp[0]*2*np.pi
            theta = samp[1]*2*np.pi
            r = samp[2] #*self.sphere_r
            samp[0] = r * np.sin(theta) * np.cos(phi)
            samp[1] = r * np.sin(theta) * np.sin(phi)
            samp[2] = r * np.cos(theta)
        else:
            samp = np.random.rand(3,)
            samp[0] = samp[0]*(self.x_max-self.x_min) + self.x_min
            samp[1] = samp[1]*(self.y_max-self.y_min) + self.y_min
            samp[2] = samp[2]*(self.z_max-self.z_min) + self.z_min
        self.sample_unit_sphere()
        # samp = np.random.rand(3,)
        # samp[0] = samp[0]*(self.x_max-self.x_min) + self.x_min
        # samp[1] = samp[1]*(self.y_max-self.y_min) + self.y_min
        # samp[2] = samp[2]*(self.z_max-self.z_min) + self.z_min
        transformedsamp = np.matmul(self.transform, np.array([samp[0], samp[1], samp[2], 1]).reshape(4,1))
        transformedsamp = np.array([transformedsamp[0], transformedsamp[1], transformedsamp[2]]).reshape((3,))
        return transformedsamp
    
    def sample_unit_sphere(self):
        """
        """
        samp = np.random.normal(size=(3,))
        r = np.random.uniform()
        samp = samp * (r/np.linalg.norm(samp))
        return samp 

    def sample_sphere_intersects_trumpet(self, start:SteerableNeedle):
        """
        """           
        valid = False
        while (not valid):
            unit_samp = self.sample_unit_sphere()
            samp_ = start.needle_lims[0,1]*unit_samp

            samp = start.p + samp_
            if start.get_distance(samp) < start.needle_lims[0,1]:
                # print(f"new sample: {samp} samp: {samp_} ")
                valid = True
                return samp

    def test_collisions_world(self, p) -> bool:
        """
        Test collision for the robot position p.

        Parameters:
        p (3x1): world coordinate point to test

        Returns:
        collisions (bool): True if in collision, False if not

        """
        pvox = np.matmul(self.transforminv, np.array([p[0], p[1], p[2], 1]).reshape(4,1))
        inds = np.floor(pvox).astype(int)


        if pvox[0] <= self.x_min or pvox[0] >= self.x_max or pvox[1] <= self.y_min or pvox[1] >= self.y_max or pvox[2] <= self.z_min or pvox[2] >= self.z_max:
        #    print("THIS HAPPENED")
           return True # test for out of bounds, mightnot be necessary if handled elsewhere. SIMON CODE
        # if _DEBUG:
        #     print(f"p: {p} pvox: {pvox[0:3].reshape(-1,)} val: {self.voxel_grid[inds[0], inds[1], inds[2]]}")
        collisions = self.voxel_grid[inds[0], inds[1], inds[2]] == _OCCUPIED
        if collisions:
            print(f"collision at {p.reshape(-1,)}  inds: {inds.reshape(-1,)}")
        return collisions
    

    def convert_to_voxel(self, p):
        """
        Convert world points to voxel coordinates.

        Parameters:
        p (3xn): n sets of points in world space

        Returns:
        pvox (nx3): n sets of voxel coordinates 

        """
        points = np.array(p)
        points = points.reshape(3,-1)
        numpoints = np.shape(points)
        bulkpoints = np.vstack((points, np.ones((1,numpoints[1])))).reshape(4,-1)
        pvox = np.matmul(self.transforminv, bulkpoints)
        pvox = pvox[0:3,:].reshape(-1,3)

        return pvox


    def convert_to_world(self, pvox):
        """
        Convert voxel coordinates to world points.

        Parameters:
        p (3xn): n sets of points in voxel coordinates

        Returns:
        pvox (nx3): n sets of world space

        """
        points = np.array(pvox)
        points = points.reshape(3,-1)
        numpoints = np.shape(points)
        bulkpoints = np.vstack((points, np.ones((1,numpoints[1])))).reshape(4,-1)
        p = np.matmul(self.transform, bulkpoints)
        p = p[0:3,:].reshape(3,-1)

        return p


    def draw_path(self, plan, planner, dynamic_tree=False, dynamic_plan=True, show=True, filename=None, title=''):
        """
        Draw the environment with an overlaid plan.

        Parameters:
        plan (): sequence of configurations to be drawn as plan (not drawn if pass in None)
        planner (): a planner which has a function of the form
                  vertices, edges = planner.T.get_states_and_edges()
                  if None the search graph is not drawn
        
        """
        
        
        ax = plt.figure().add_subplot(projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')

        if self.sphere_center is not None:
            # ax.set_xlim(self.sphere_center[0] - self.sphere_r, self.sphere_center[0] + self.sphere_r)
            # ax.set_ylim(self.sphere_center[1] - self.sphere_r, self.sphere_center[1] + self.sphere_r)
            # ax.set_zlim(self.sphere_center[2] - self.sphere_r, self.sphere_center[2] + self.sphere_r) 
            corners = np.array([[self.x_min, self.x_min, self.x_min, self.x_max, self.x_min, self.x_max, self.x_max, self.x_max], [self.y_min, self.y_min, self.y_max, self.y_min, self.y_max, self.y_min, self.y_max, self.y_max], [self.z_min, self.z_max, self.z_min, self.z_min, self.z_max, self.z_max, self.z_min, self.z_max]]).reshape((3,-1))
            print(corners)
            world_corners = self.convert_to_world(corners)
            print(world_corners)
            ax.set_xlim(np.min(world_corners[0,:]), np.max(world_corners[0,:]))
            ax.set_ylim(np.min(world_corners[1,:]), np.max(world_corners[1,:]))
            ax.set_zlim(np.min(world_corners[2,:]), np.max(world_corners[2,:])) 


        else:
            corners = np.array([[self.x_min, self.x_min, self.x_min, self.x_max, self.x_min, self.x_max, self.x_max, self.x_max], [self.y_min, self.y_min, self.y_max, self.y_min, self.y_max, self.y_min, self.y_max, self.y_max], [self.z_min, self.z_max, self.z_min, self.z_min, self.z_max, self.z_max, self.z_min, self.z_max]]).reshape((3,-1))
            world_corners = self.convert_to_world(corners)

            ax.set_xlim(np.min(world_corners[0,:]), np.max(world_corners[0,:]))
            ax.set_ylim(np.min(world_corners[1,:]), np.max(world_corners[1,:]))
            ax.set_zlim(np.min(world_corners[2,:]), np.max(world_corners[2,:]))   

            # ax.set_xlim(np.min(world_corners[0,:]), np.min(world_corners[0,:]))
            # ax.set_ylim(self.y_min, self.y_max)
            # ax.set_zlim(self.z_min, self.z_max)      

        plt.title(title)
        ws_goal = self.goal
        ws_init = self.start
        ax.scatter(ws_goal[0], ws_goal[1], ws_goal[2], c='#CC6677')
        ax.scatter(ws_init[0], ws_init[1], ws_init[2], c='#F7AD03')
        print(f"start: {ws_init} goal: {ws_goal}")
        # plt.show()
        # plt.pause(0.001)


        if planner is not None:
            Qs, edges = planner.T.get_states_and_edges()
            # Draw tree for the needle
            for i, e in enumerate(edges):
                e0 = e[0]
                e1 = e[1]
                ax.plot([e0[0], e1[0]], [e0[1], e1[1]], [e0[2], e1[2]], c='#332288')
                if dynamic_tree:
                    plt.pause(0.001)

        if plan is not None:
            # Draw plan found to goal for the needle
            for i in range(1,len(plan)):
                Qp = plan[i-1]
                Qr = plan[i]
                Ps = Qp
                Rs = Qr
                ax.plot([Ps[0], Rs[0]], [Ps[1],Rs[1]], [Ps[2],Rs[2]], c='#44AA99')
                if dynamic_plan:
                    plt.pause(0.001)

        if show:
            plt.show()
            
        if filename is not None:
            plt.savefig(filename,bbox_inches='tight')
            plt.pause(0.5)
            plt.close()


    def export_rrt(self, states, path, filename, color, selectColor, activeColor):
        """
        Exports planning data to json file for visualization later. 
        """
        # path = path.tolist()
        # states = states.tolist()
        data = []
        index = 1
        if states is not None: 
            for point in states:
                # print(point)
                data.append({"label": f"F-{index}", "position": point.tolist(), "selected": False, "locked": True})
                index += 1

        if path is not None:
            for point in path:
                data.append({"label": f"F-{index}", "position": point.tolist(), "selected": True, "locked": True}) 
                index += 1

        fileinfo = {"@schema": "https://raw.githubusercontent.com/Slicer/Slicer/main/Modules/Loadable/Markups/Resources/Schema/markups-schema-v1.0.0.json#",
                    "markups": [{"type": "Fiducial", "coordinateSystem": "RAS", "controlPoints": data, 
                                "display": {"opacity": 0.5, "color": color, "selectedColor": selectColor, "activeColor": activeColor, "textScale": 0.0, "glyphSize": 1.0, "useGlyphScale": False}}]}
        with open(filename, "w") as file:
            json.dump(fileinfo, file)
