import numpy as np
# from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
import matplotlib.pyplot as plt
 
src = None
erosion_size = 0
max_elem = 2
max_kernel_size = 21
title_trackbar_element_shape = 'Element:\n 0: Rect \n 1: Cross \n 2: Ellipse'
title_trackbar_kernel_size = 'Kernel size:\n 2n +1'
title_erosion_window = 'Erosion Demo'
title_dilation_window = 'Dilation Demo'



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
        obstacles_np = np.where(obstacles_np > 1, 0, obstacles_np)
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

        obstacle_coords = np.where(self.voxel_grid == 1)
        obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()

        f = open(f"{filename}.txt", 'a')
        np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
        np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
        obstacle_arr_full = self.get_obstacles_downsampled(obstacle_arr)
        np.savetxt(f, obstacle_arr_full, fmt='%d', delimiter=" ")
        f.close()

        f = open(f"{filename}_outline.txt", 'a')
        np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
        np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
        obstacle_arr_outline = self.get_obstacles_outline(obstacle_arr)
        np.savetxt(f, obstacle_arr_outline, fmt='%d', delimiter=" ")
        f.close()

        f = open(f"{filename}_outline_speckled.txt", 'a')
        np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
        np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
        obstacle_arr_outline_speckled = self.get_obstacles_outline_speckled(obstacle_arr)
        np.savetxt(f, obstacle_arr_outline_speckled, fmt='%d', delimiter=" ")
        f.close()

        f = open(f"{filename}_outline_viz.txt", 'a')
        np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
        np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
        obstacle_arr_outline_viz = self.get_obstacles_outline_viz(obstacle_arr)
        np.savetxt(f, obstacle_arr_outline_viz, fmt='%d', delimiter=" ")
        f.close()


    def get_obstacles_full(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        return obstacle_arr


    def get_obstacles_isosurface(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        obstacle_coordsx = np.where(np.logical_and(obstacle_arr[:,0] > 50, obstacle_arr[:,0] < self.x_max - 25))
        obstacle_arr = obstacle_arr[obstacle_coordsx[0],:]
        obstacle_coordsy = np.where(np.logical_and(obstacle_arr[:,1] > 50, obstacle_arr[:,1] < self.y_max - 25))
        obstacle_arr = obstacle_arr[obstacle_coordsy[0],:]
        obstacle_coordsz = np.where(np.logical_and(obstacle_arr[:,2] > 30, obstacle_arr[:,2] < self.z_max - 75))
        obstacle_arr = obstacle_arr[obstacle_coordsz[0],:]
        return obstacle_arr
    

    def get_obstacles_outline(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        mask = np.zeros_like(self.voxel_grid)
        for i in range(0,int(self.x_max)):
            # plt.imshow(self.voxel_grid[i,:,:])
            # plt.show()
            eroded_mask = np.logical_not(get_shell(self.voxel_grid[i,:,:]))
            # plt.imshow(eroded_mask)
            # plt.show()
            mask[i,:,:] = np.logical_and(self.voxel_grid[i,:,:], eroded_mask)
            # plt.imshow(mask[i,:,:])
            # plt.show()
        obstacle_coords = np.where(mask == 1)
        obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
        return obstacle_arr


    def get_obstacles_outline_speckled(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        mask = np.zeros_like(self.voxel_grid)
        for i in range(0,int(self.x_max)):
            # plt.imshow(self.voxel_grid[i,:,:])
            # plt.show()
            eroded_mask = np.logical_not(get_shell(self.voxel_grid[i,:,:]))
            # plt.imshow(eroded_mask)
            # plt.show()
            mask[i,:,:] = np.logical_and(self.voxel_grid[i,:,:], eroded_mask)
            # plt.imshow(mask[i,:,:])
            # plt.show()
        obstacle_coords = np.where(mask == 1)
        obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
        return obstacle_arr


    def get_obstacles_outline_viz(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        mask = np.zeros_like(self.voxel_grid)
        for i in range(0,int(self.x_max)):
            # plt.imshow(self.voxel_grid[i,:,:])
            # plt.show()
            eroded_mask = np.logical_not(remove_shell(self.voxel_grid[i,:,:]))
            # plt.imshow(eroded_mask)
            # plt.show()
            mask[i,:,:] = np.logical_and(self.voxel_grid[i,:,:], eroded_mask)
            # plt.imshow(mask[i,:,:])
            # plt.show()
        obstacle_coords = np.where(mask == 1)
        obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
        return obstacle_arr


    def get_obstacles_truncated(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        obstacle_coordsx = np.where(np.logical_and(obstacle_arr[:,0] > 50, obstacle_arr[:,0] < self.x_max - 25))
        obstacle_arr = obstacle_arr[obstacle_coordsx[0],:]
        obstacle_coordsy = np.where(np.logical_and(obstacle_arr[:,1] > 50, obstacle_arr[:,1] < self.y_max - 25))
        obstacle_arr = obstacle_arr[obstacle_coordsy[0],:]
        obstacle_coordsz = np.where(np.logical_and(obstacle_arr[:,2] > 30, obstacle_arr[:,2] < self.z_max - 75))
        obstacle_arr = obstacle_arr[obstacle_coordsz[0],:]
        return obstacle_arr


    def get_obstacles_downsampled(self, obstacle_arr):
        """
        Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
        """
        arr_length = np.shape(obstacle_arr)[0]
        obstacle_sub = np.random.random_integers(0,arr_length,arr_length//8)
        obstacle_arr = obstacle_arr[obstacle_sub,:]
        return obstacle_arr







 
 
# from the opencv demo https://docs.opencv.org/4.x/db/df6/tutorial_erosion_dilatation.html  
def main(image):
    global src
    uint_img = np.array(image*255).astype('uint8')
    src = cv.cvtColor(uint_img, cv.COLOR_GRAY2BGR)
    if src is None:
        print('Could not open or find the image: ', image)
        exit(0)
 
    # cv.namedWindow(title_erosion_window)
    # cv.createTrackbar(title_trackbar_element_shape, title_erosion_window, 0, max_elem, erosion)
    # cv.createTrackbar(title_trackbar_kernel_size, title_erosion_window, 0, max_kernel_size, erosion)
 
    # cv.namedWindow(title_dilation_window)
    # cv.createTrackbar(title_trackbar_element_shape, title_dilation_window, 0, max_elem, dilatation)
    # cv.createTrackbar(title_trackbar_kernel_size, title_dilation_window, 0, max_kernel_size, dilatation)
 
    erosion_dst = erosion(0)
    erosion_dst = np.asarray(erosion_dst)
    # print(np.shape(erosion_dst))
    erosion_dst = erosion_dst[:,:,0]//255
    # print(np.shape(erosion_dst))
    return erosion_dst
    # dilatation(0)
    # cv.waitKey()


# from the opencv demo https://docs.opencv.org/4.x/db/df6/tutorial_erosion_dilatation.html  
def get_shell(image):
    global src
    uint_img = np.array(image*255).astype('uint8')
    src = cv.cvtColor(uint_img, cv.COLOR_GRAY2BGR)
    if src is None:
        print('Could not open or find the image: ', image)
        exit(0)
 
    erosion_dst = erosion(0, erosion_size=5)
    erosion_dst = np.asarray(erosion_dst)
    # print(np.shape(erosion_dst))
    erosion_dst = erosion_dst[:,:,0]//255
    # print(np.shape(erosion_dst))
    return erosion_dst



def remove_shell(image):
    global src
    uint_img = np.array(image*255).astype('uint8')
    src = cv.cvtColor(uint_img, cv.COLOR_GRAY2BGR)
    if src is None:
        print('Could not open or find the image: ', image)
        exit(0)
 
    src = erosion(0, erosion_size=5)
    dilation_dst = dilatation(0, dilatation_size=7)
    morph_dst = np.asarray(dilation_dst)
    morph_dst = morph_dst[:,:,0]//255
    return morph_dst
 
 
# optional mapping of values with morphological shapes
def morph_shape(val):
    if val == 0:
        return cv.MORPH_RECT
    elif val == 1:
        return cv.MORPH_CROSS
    elif val == 2:
        return cv.MORPH_ELLIPSE
 
 
 
def erosion(val, erosion_size = 1):
    erosion_shape = morph_shape(val)
    element = cv.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                       (erosion_size, erosion_size))
    erosion_dst = cv.erode(src, element)
    return erosion_dst
 
 
def dilatation(val, dilatation_size=1):
    dilation_shape = morph_shape(val)
    element = cv.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
    dilatation_dst = cv.dilate(src, element)
    return dilatation_dst
 
 
 
if __name__ == "__main__":
    # parser = argparse.ArgumentParser(description='Code for Eroding and Dilating tutorial.')
    # parser.add_argument('--input', help='Path to input image.', default='LinuxLogo.jpg')
    # args = parser.parse_args()
 
    # main(args.input)


    envparser = ReMINDEnvironment()
    envparser.read_env("./../data/input/ReMIND_info_001.txt")
    envparser.write_obstacles("./../data/input/remind_obstacles")



# TODO: write the segmentations to a text file for reasonable visualizations post planning