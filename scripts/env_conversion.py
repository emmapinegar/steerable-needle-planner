import numpy as np


_DEBUG = False
_BOUNDS = 'Bounds:'
_GOAL = 'Goal:'
_SPHERE = 'Sphere:'
_START = 'Start:'
_NEEDLE = 'NeedleRobot:'
_POSE = 'Pose:'
_OBSTACLES = 'Obstacles:'
_SPHERE_BOUNDS = "SphereBounds:"
_TRANSFORM = "Transform:"
_CENTERSLICE = "CenterSlice:"


class ReMINDEnvironment:
    """
    A class to handle the concerns of the planning environments from the ReMIND dataset.
    """

    def __init__(self):
        self.voxel_grid = None
        self.goal = [0, 0, 0]
        self.start = [0, 0, 0]
        self.gw = np.eye(4)
        self.transform = np.eye(4)
        self.sphere_center = None
        self.line_parser = {_BOUNDS: self.parse_bounds,
                            _NEEDLE: self.parse_needle,
                            _GOAL: self.parse_goal,
                            _OBSTACLES: self.parse_obstacles,
                            _START: self.parse_start,
                            _SPHERE_BOUNDS: self.parse_sphere_bounds,
                            _TRANSFORM: self.parse_transform,
                            }


    def read_env(self, env_file_path):
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
        for l in file_infos:
            line_info = l.strip().split()
            if line_info[0].startswith('#'):
                continue
            self.line_parser[line_info[0]](line_info[1:])

        self.transforminv = np.linalg.inv(self.transform)


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


    def parse_sphere_bounds(self, line_data):
        """
        Parse sphere bounds of environment.
        """
        self.sphere_center = [float(line_data[0]), float(line_data[1]), float(line_data[2])]
        self.sphere_r = float(line_data[3])


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
            self.goal = np.array([float(l) for l in line_data])


    def parse_start(self, line_data):
        """
        Parse start location.
        """
        if self.start is not None:
            self.gw = np.array([float(l) for l in line_data]).reshape(-1,4)
            self.start = self.gw[0:3, 3].reshape(3,1)


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



    def write_obstacles(self, filename):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """

        f = open(filename, 'a')
        np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")

        np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
        obstacle_coords = np.where(self.voxel_grid == 1)
        obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
        # print(obstacle_arr[:,0])
        obstacle_coordsx = np.where(np.logical_and(obstacle_arr[:,0] > 150, obstacle_arr[:,0] < self.x_max - 50))
        obstacle_arr = obstacle_arr[obstacle_coordsx[0],:]
        # print(obstacle_coordsx)
        # print(obstacle_arr)
        obstacle_coordsy = np.where(np.logical_and(obstacle_arr[:,1] > 150, obstacle_arr[:,1] < self.y_max - 50))
        obstacle_arr = obstacle_arr[obstacle_coordsy[0],:]
        # print(obstacle_arr)
        obstacle_coordsz = np.where(np.logical_and(obstacle_arr[:,2] > 30, obstacle_arr[:,2] < self.z_max - 75))
        obstacle_arr = obstacle_arr[obstacle_coordsz[0],:]
        arr_length = np.shape(obstacle_arr)[0]
        obstacle_sub = np.random.random_integers(0,arr_length,arr_length//4)
        obstacle_arr = obstacle_arr[obstacle_sub,:]
        # print(obstacle_arr)
        np.savetxt(f, obstacle_arr, fmt='%d', delimiter=" ")

        f.close()


    def write_start_and_goal():
        """
        
        """




if __name__=="__main__":

    envparser = ReMINDEnvironment()
    envparser.read_env("./../data/input/ReMIND_info_001.txt")
    envparser.write_obstacles("./../data/input/remind_obstacles.txt")



# TODO: write the segmentations to a text file for reasonable visualizations post planning