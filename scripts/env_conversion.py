# import numpy as np
# # from __future__ import print_function
# import cv2 as cv
# import numpy as np
# import argparse
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation as R
 
# src = None
# erosion_size = 0
# max_elem = 2
# max_kernel_size = 21
# title_trackbar_element_shape = 'Element:\n 0: Rect \n 1: Cross \n 2: Ellipse'
# title_trackbar_kernel_size = 'Kernel size:\n 2n +1'
# title_erosion_window = 'Erosion Demo'
# title_dilation_window = 'Dilation Demo'

# _GOAL = 5
# _START = 4
# _TUMOR = 3
# _BRAIN = 2
# _VENTRICLES = 1

# _DEBUG = False
# _BOUNDS = 'Bounds:'
# _GOAL = 'Goal:'
# _SPHERE = 'Sphere:'
# _START = 'Start:'
# _NEEDLE = 'NeedleRobot:'
# _POSE = 'Pose:'
# _OBSTACLES = 'Obstacles:'
# _SPHERE_BOUNDS = "SphereBounds:"
# _TRANSFORM = "Transform:"
# _CENTERSLICE = "CenterSlice:"


# class ReMINDEnvironment:
#     """
#     A class to handle the concerns of the planning environments from the ReMIND dataset.
#     """

#     def __init__(self):
#         self.voxel_grid = None
#         self.goal = [0, 0, 0]
#         self.start = [0, 0, 0]
#         self.gw = np.eye(4)
#         self.transform = np.eye(4)
#         self.sphere_center = None
#         self.line_parser = {_BOUNDS: self.parse_bounds,
#                             _NEEDLE: self.parse_needle,
#                             _GOAL: self.parse_goal,
#                             _OBSTACLES: self.parse_obstacles,
#                             _START: self.parse_start,
#                             _SPHERE_BOUNDS: self.parse_sphere_bounds,
#                             _TRANSFORM: self.parse_transform,
#                             }


#     def read_env(self, env_file_path):
#         """
#         Reads in the information about the environment when the file conforms to the following format:
#         <typename>: values
#         Bounds: x_min x_max y_min y_max z_min z_max
#         SphereBounds: x_center y_center z_center radius
#         Start: T_xx T_yx T_zx T_dx T_xy T_yy T_zy T_dy T_xz T_yz T_zz T_dz 0 0 0 1
#         Goal: x_goal y_goal z_goal
#         Obstacles: obstacle_file_name.npy
#         Segmentations: segmentation_file_name.npy
#         NeedleRobot: el_min el_max k_min k_max phi_min phi_max
#         Transform: T_xx T_yx T_zx T_dx T_xy T_yy T_zy T_dy T_xz T_yz T_zz T_dz 0 0 0 1
#         CenterSlice: x_slice y_slice z_slice

#         """
#         env_file = open(env_file_path, 'r')
#         file_infos = env_file.readlines()
#         for l in file_infos:
#             line_info = l.strip().split()
#             if line_info[0].startswith('#'):
#                 continue
#             self.line_parser[line_info[0]](line_info[1:])

#         self.transforminv = np.linalg.inv(self.transform)


#     def parse_bounds(self, line_data):
#         """
#         Parse environment bounds.
#         """
#         self.x_min = float(line_data[0])
#         self.x_max = float(line_data[1])
#         self.y_min = float(line_data[2])
#         self.y_max = float(line_data[3])
#         self.z_min = float(line_data[4])
#         self.z_max = float(line_data[5])
#         self.lims = np.array([[self.x_min, self.x_max], [self.y_min, self.y_max], [self.z_min, self.z_max]])


#     def parse_sphere_bounds(self, line_data):
#         """
#         Parse sphere bounds of environment.
#         """
#         self.sphere_center = [float(line_data[0]), float(line_data[1]), float(line_data[2])]
#         self.sphere_r = float(line_data[3])


#     def parse_obstacles(self, line_data):
#         """
#         Parse file containing obstacle data.
#         """
#         obstacle_file = str(line_data[0])
#         obstacles_np = np.load(obstacle_file)
#         spread = 5
#         xstart = 280 #np.floor(self.start[0,0]).astype(int)
#         ystart = 215 #np.floor(self.start[1,0]).astype(int)
#         zstart = 50 #np.floor(self.start[2,0]).astype(int)
#         # transformedstart = np.matmul(self.transform, np.array([xstart, ystart, zstart, 1]).reshape(4,1))
#         # transformedstartinv = np.matmul(self.transforminv, np.array([xstart, ystart, zstart, 1]).reshape(4,1))
#         # print(transformedstart)
#         # print(transformedstartinv)
#         # xgoal = np.floor(self.goal[0]).astype(int)
#         # ygoal = np.floor(self.goal[1]).astype(int)
#         # zgoal = np.floor(self.goal[2]).astype(int)

#         # print(f"goal x: {xstart}   y: {ystart}   z: {zstart} \t goal x: {xgoal}   y: {ygoal}   z: {zgoal}")

#         # obstaclefilename = obstacle_file.replace("-", "_obstacles_")
#         # print(obstacles_np[xstart-spread:xstart+spread, ystart-spread:ystart+spread, zstart-spread:zstart+spread])
#         obstacles_np[xstart-spread:xstart+spread, ystart-spread:ystart+spread, zstart-spread//2:zstart+spread] = 0
#         # # obstacles_np[xgoal-spread:xgoal+spread, ygoal-spread:ygoal+spread, zgoal-spread:zgoal+spread] = _BRAIN
#         # print(np.shape(obstacles_np))
#         # obstacles = np.logical_or(np.logical_not(obstacles_np == _BRAIN), obstacles_np == _VENTRICLES)
#         # print(np.shape(obstacles))
#         # obstacles = np.logical_and(np.logical_not(obstacles_np == _TUMOR), obstacles).astype(int)
#         # print(np.shape(obstacles))
#         # np.save(obstaclefilename, obstacles)
#         # for i in range(200,210):#int(self.x_max)):
#         #     plt.imshow(obstacles_np[i,:,:])
#         #     plt.show()
#         # print(np.where(obstacles_np > 1))
#         # print(obstacles_np[350, 215, 40])
#         obstacles_np = np.where(obstacles_np > 1, 0, obstacles_np)
#         self.voxel_grid = obstacles_np
        


#     def parse_goal(self, line_data):
#         """
#         Parse goal location.
#         """
#         if self.goal is not None:
#             self.goal = np.array([float(l) for l in line_data])


#     def parse_start(self, line_data):
#         """
#         Parse start location.
#         """
#         if self.start is not None:
#             self.gw = np.array([float(l) for l in line_data]).reshape(-1,4)
#             self.start = self.gw[0:3, 3].reshape(3,1)


#     def parse_needle(self, line_data):
#         """
#         Parse parameters for steerable needle robot.
#         """
#         self.needle_lims = np.array([float(l) for l in line_data]).reshape(-1,2)


#     def parse_transform(self, line_data):
#         """
#         Parse transformation matrix that converts from pixel indices to world coordinates.
#         """
#         self.transform = np.array([float(l) for l in line_data]).reshape(-1,4)
#         print(f"transform: \n{self.transform}")


#     def write_obstacles(self, filename):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """

#         xstart = np.floor(self.start[0,0]).astype(int)
#         ystart = np.floor(self.start[1,0]).astype(int)
#         zstart = np.floor(self.start[2,0]).astype(int)
#         transformedstart = np.matmul(self.transform, np.array([xstart, ystart, zstart, 1]).reshape(4,1))
#         transformedstartinv = np.matmul(self.transforminv, np.array([xstart, ystart, zstart, 1]).reshape(4,1))
#         transformedgoal = np.matmul(self.transform, np.array([np.floor(self.goal[0]).astype(int), np.floor(self.goal[1]).astype(int), np.floor(self.goal[2]).astype(int), 1]).reshape(4,1))

#         start_transformation = np.eye(3,3)
#         start_transformation[0:3,0] = np.divide(self.transform[0:3,0], np.linalg.norm(self.transform[0:3,0]))
#         start_transformation[0:3,1] = np.divide(self.transform[0:3,1], np.linalg.norm(self.transform[0:3,1]))
#         start_transformation[0:3,2] = np.divide(self.transform[0:3,2], np.linalg.norm(self.transform[0:3,2]))

#         # https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.html

#         r = R.from_matrix([[start_transformation[0,0], start_transformation[0,1], start_transformation[0,2]], [start_transformation[1,0], start_transformation[1,1], start_transformation[1,2]], [start_transformation[2,0], start_transformation[2,1], start_transformation[2,2]]])
#         start_quat = r.as_quat()
#         print(f"start: {transformedstart[0,0]} {transformedstart[1,0]} {transformedstart[2,0]} {start_quat[1]} {start_quat[2]} {start_quat[3]} {start_quat[0]}")
#         print(f"goal: {transformedgoal[0,0]} {transformedgoal[1,0]} {transformedgoal[2,0]}")
#         obstacle_coords = np.where(self.voxel_grid == 1)
#         # print(np.shape(obstacle_coords))
#         obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()

#         # f = open(f"{filename}.txt", 'a')
#         # np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
#         # np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
#         # obstacle_arr_full = self.get_obstacles_downsampled(obstacle_arr)
#         # np.savetxt(f, obstacle_arr_full, fmt='%d', delimiter=" ")
#         # f.close()
#         obstacle_arr_outline = self.get_obstacles_outline(obstacle_arr)

#         f = open(f"{filename}_outline.txt", 'a')
#         np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
#         np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
#         np.savetxt(f, obstacle_arr_outline, fmt='%d', delimiter=" ")
#         f.close()

#         np.random.shuffle(obstacle_arr_outline)
#         np.random.shuffle(obstacle_arr_outline)
#         print(np.shape(obstacle_arr_outline))
#         f = open(f"{filename}_outline_shuffled.txt", 'a')
#         np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
#         np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
#         np.savetxt(f, obstacle_arr_outline, fmt='%d', delimiter=" ")
#         f.close()

#         # f = open(f"{filename}_outline_downsampled.txt", 'a')
#         # np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
#         # np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
#         # obstacle_arr_outline_downsampled = self.get_obstacles_downsampled(obstacle_arr_outline)
#         # np.savetxt(f, obstacle_arr_outline_downsampled, fmt='%d', delimiter=" ")
#         # f.close()



#         # f = open(f"{filename}_outline_speckled.txt", 'a')
#         # np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
#         # np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
#         # obstacle_arr_outline_speckled = self.get_obstacles_outline_speckled(obstacle_arr)
#         # np.savetxt(f, obstacle_arr_outline_speckled, fmt='%d', delimiter=" ")
#         # f.close()

#         f = open(f"{filename}_outline_viz.txt", 'a')
#         np.savetxt(f, self.transform, fmt='%1.4f', newline="\n")
#         np.savetxt(f, np.array([self.x_max, self.y_max, self.z_max]).reshape(1,-1), fmt='%d', delimiter=" ")
#         obstacle_arr_outline_viz = self.get_obstacles_outline_viz(obstacle_arr)
#         np.savetxt(f, obstacle_arr_outline_viz, fmt='%d', delimiter=" ")
#         f.close()


#     def get_obstacles_full(self, obstacle_arr):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """
#         return obstacle_arr


#     def get_obstacles_isosurface(self, obstacle_arr):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """
#         obstacle_coordsx = np.where(np.logical_and(obstacle_arr[:,0] > 50, obstacle_arr[:,0] < self.x_max - 25))
#         obstacle_arr = obstacle_arr[obstacle_coordsx[0],:]
#         obstacle_coordsy = np.where(np.logical_and(obstacle_arr[:,1] > 50, obstacle_arr[:,1] < self.y_max - 25))
#         obstacle_arr = obstacle_arr[obstacle_coordsy[0],:]
#         obstacle_coordsz = np.where(np.logical_and(obstacle_arr[:,2] > 30, obstacle_arr[:,2] < self.z_max - 75))
#         obstacle_arr = obstacle_arr[obstacle_coordsz[0],:]
#         return obstacle_arr
    

#     def get_obstacles_outline(self, obstacle_arr):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """
#         mask = np.zeros_like(self.voxel_grid)
#         for i in range(0,int(self.x_max)):
#             eroded_mask = np.logical_not(get_shell(self.voxel_grid[i,:,:]))
#             mask[i,:,:] = np.logical_and(self.voxel_grid[i,:,:], eroded_mask)
#             if i == self.start[0,0]:
#                 plt.imshow(self.voxel_grid[i,:,:])
#                 plt.show()
#                 plt.imshow(eroded_mask)
#                 plt.show()
#                 plt.imshow(mask[i,:,:])
#                 plt.show()
#         obstacle_coords = np.where(mask == 1)
#         obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
#         return obstacle_arr


#     def get_obstacles_outline_speckled(self, obstacle_arr):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """
#         mask = np.zeros_like(self.voxel_grid)
#         for i in range(0,int(self.x_max)):
#             # plt.imshow(self.voxel_grid[i,:,:])
#             # plt.show()
#             eroded_mask = np.logical_not(get_shell(self.voxel_grid[i,:,:]))
#             arr_shape = np.shape(eroded_mask)
#             obstacle_x = np.random.random_integers(0,arr_shape[0]-1,(arr_shape[0]*arr_shape[1])//32)
#             obstacle_y = np.random.random_integers(0,arr_shape[1]-1,(arr_shape[0]*arr_shape[1])//32)
#             # print(np.shape(eroded_mask))
#             eroded_mask[obstacle_x, obstacle_y] = 1.0
#             # plt.imshow(eroded_mask)
#             # plt.show() 
#             mask[i,:,:] = np.logical_and(self.voxel_grid[i,:,:], eroded_mask)
#             # plt.imshow(mask[i,:,:])
#             # plt.show()
        
#         obstacle_coords = np.where(mask == 1)
#         obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
#         return obstacle_arr


#     def get_obstacles_outline_viz(self, obstacle_arr):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """
#         mask = np.zeros_like(self.voxel_grid)
#         for i in range(0,int(self.x_max)):
#             # plt.imshow(self.voxel_grid[i,:,:])
#             # plt.show()
#             eroded_mask = np.logical_not(remove_shell(self.voxel_grid[i,:,:]))
#             # plt.imshow(eroded_mask)
#             # plt.show()
#             mask[i,:,:] = np.logical_and(self.voxel_grid[i,:,:], eroded_mask)
#             # plt.imshow(mask[i,:,:])
#             # plt.show()
#         obstacle_coords = np.where(mask == 1)
#         obstacle_arr = np.array((obstacle_coords[0], obstacle_coords[1], obstacle_coords[2])).transpose()
#         return obstacle_arr


#     def get_obstacles_downsampled(self, obstacle_arr):
#         """
#         Write the obstacle file to a text file with the transformation matrix preceeding the obstacle voxel coordinates.
#         """
#         arr_length = np.shape(obstacle_arr)[0]
#         obstacle_sub = np.random.random_integers(0,arr_length-1,(31*arr_length)//32)
#         obstacle_arr = obstacle_arr[obstacle_sub,:]
#         return obstacle_arr







 
 
# # from the opencv demo https://docs.opencv.org/4.x/db/df6/tutorial_erosion_dilatation.html  
# def main(image):
#     global src
#     uint_img = np.array(image*255).astype('uint8')
#     src = cv.cvtColor(uint_img, cv.COLOR_GRAY2BGR)
#     if src is None:
#         print('Could not open or find the image: ', image)
#         exit(0)
 
#     # cv.namedWindow(title_erosion_window)
#     # cv.createTrackbar(title_trackbar_element_shape, title_erosion_window, 0, max_elem, erosion)
#     # cv.createTrackbar(title_trackbar_kernel_size, title_erosion_window, 0, max_kernel_size, erosion)
 
#     # cv.namedWindow(title_dilation_window)
#     # cv.createTrackbar(title_trackbar_element_shape, title_dilation_window, 0, max_elem, dilatation)
#     # cv.createTrackbar(title_trackbar_kernel_size, title_dilation_window, 0, max_kernel_size, dilatation)
 
#     erosion_dst = erosion(0)
#     erosion_dst = np.asarray(erosion_dst)
#     # print(np.shape(erosion_dst))
#     erosion_dst = erosion_dst[:,:,0]//255
#     # print(np.shape(erosion_dst))
#     return erosion_dst
#     # dilatation(0)
#     # cv.waitKey()


# # from the opencv demo https://docs.opencv.org/4.x/db/df6/tutorial_erosion_dilatation.html  
# def get_shell(image):
#     global src
#     uint_img = np.array(image*255).astype('uint8')
#     src = cv.cvtColor(uint_img, cv.COLOR_GRAY2BGR)
#     if src is None:
#         print('Could not open or find the image: ', image)
#         exit(0)
 
#     erosion_dst = erosion(0, erosion_size=1)
#     erosion_dst = np.asarray(erosion_dst)
#     # print(np.shape(erosion_dst))
#     erosion_dst = erosion_dst[:,:,0]//255
#     # print(np.shape(erosion_dst))
#     return erosion_dst



# def remove_shell(image):
#     global src
#     uint_img = np.array(image*255).astype('uint8')
#     src = cv.cvtColor(uint_img, cv.COLOR_GRAY2BGR)
#     if src is None:
#         print('Could not open or find the image: ', image)
#         exit(0)
 
#     src = erosion(0, erosion_size=5)
#     dilation_dst = dilatation(0, dilatation_size=7)
#     morph_dst = np.asarray(dilation_dst)
#     morph_dst = morph_dst[:,:,0]//255
#     return morph_dst


# # optional mapping of values with morphological shapes
# def morph_shape(val):
#     if val == 0:
#         return cv.MORPH_RECT
#     elif val == 1:
#         return cv.MORPH_CROSS
#     elif val == 2:
#         return cv.MORPH_ELLIPSE


# def erosion(val, erosion_size = 2):
#     erosion_shape = morph_shape(val)
#     element = cv.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
#                                        (erosion_size, erosion_size))
#     erosion_dst = cv.erode(src, element)
#     return erosion_dst


# def dilatation(val, dilatation_size=1):
#     dilation_shape = morph_shape(val)
#     element = cv.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
#                                        (dilatation_size, dilatation_size))
#     dilatation_dst = cv.dilate(src, element)
#     return dilatation_dst


# if __name__ == "__main__":
#     # parser = argparse.ArgumentParser(description='Code for Eroding and Dilating tutorial.')
#     # parser.add_argument('--input', help='Path to input image.', default='LinuxLogo.jpg')
#     # args = parser.parse_args()
 
#     # main(args.input)


#     envparser = ReMINDEnvironment()
#     envparser.read_env("./../data/input/ReMIND-008_0.txt")
#     envparser.write_obstacles("./../data/input/remind_obstacles_008")



# # TODO: write the segmentations to a text file for reasonable visualizations post planning






















"""
Env_Creator: Provided some medical scans, create a .txt representation of the environment for use in collisions.py
University of Utah
April 2023
"""
import os
import shutil
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# from collisions import MDEnvironment
# from data.MedMPD.Utilities.utilities import *  # Import all functions from utilities (we need them all)
from needle import SteerableNeedle
from environment import ReMINDEnvironment
from rrt import RRT, _REACHED

import scipy.io
import nibabel as nib
from scipy import stats

_KAPPA = 0.072
_GOAL = 6
_START = 5
_TUMOR = 4
_VENTRICLES = 3
_BRAIN = 2
_SKULL = 1
_DRAW = True
_TOCARTESIAN = False

try:
    import open3d as o3d
except Exception as e:
    print(e)
    quit(1)


def process_all_ReMIND(scanfolder, pythonenvfolder, cppenvfolder):
    """
    Processes all the ReMIND segmentations and generates the files for motion planning from them.

    Parameters:

    """ 
    segmentations = []
    filenames = []
    for dirpath, dirs, files in os.walk(scanfolder):

        for file in files:
            if file.endswith(".nii"):
                filenames += [file]
                segmentations += [os.path.join(dirpath, file)]
    files = [f for f in os.listdir(scanfolder) if f.endswith('.nii') and f.__contains__("ReMIND")]
    files.sort()
    print(files)
    print(segmentations)
    digits = [f.strip("ReMIND-sgmntaiond./") for f in filenames]
    print(digits)

    numfiles = len(filenames)

    if not os.path.exists(pythonenvfolder):
        os.makedirs(pythonenvfolder)

    for i in range(numfiles):
        scanfilename = segmentations[i]

        scannum = digits[i]


        # if not os.path.exists(obstaclefilename+".npy") or not os.path.exists(segmentationfilename) or not os.path.exists(skullsegmentationfilename) or not os.path.exists(textfilename) or not os.path.exists(startfilename):
        print(f"\nprocessing scan {scannum}")
        process_ReMIND_scan(scanfilename, pythonenvfolder, cppenvfolder, scannum)


def process_ReMIND_scan(scanfilename, pythonenvfolder, cppenvfolder, scannum, goal=None):
    """
    Processes the segmentation of a single scan from the ReMIND dataset.

    Parameters:
    scanfilename (string): name of the segmentation file to process
    skullfilename (string): name of the skull segmentation to process
    obstaclefilename (string): name of the file to save the obstacle numpy array to
    segmentationfilename (string): name of the file to save the segmentation numpy array to
    skullsegmentationfilename (string): name of the file to save the transformed skull segmentation to
    textfilename (string): name of the text file to write all the details for the motion planner to
    start (3x1): x, y, z coordinates for the starting point of the needle
    """
    pyobstaclefilename = os.path.join(pythonenvfolder, f"ReMIND_obstacles_{scannum}.npy")
    cppobstaclefilename = os.path.join(cppenvfolder, f"remind_{scannum}_obstacles.txt")
    segmentationfilename = os.path.join(pythonenvfolder, f"ReMIND_segmentation_{scannum}.npy")
    pyskullsegmentationfilename = os.path.join(pythonenvfolder, f"ReMIND_skull_{scannum}_outline_shuffled.txt")
    cppskullsegmentationfilename = os.path.join(cppenvfolder, f"remind_{scannum}_skull_outline_shuffled.txt")
    pytextfilename = os.path.join(pythonenvfolder, f"ReMIND_info_{scannum}.txt")
    cppstartgoaltext = os.path.join(cppenvfolder, f"remind_{scannum}_start_and_goal_poses.txt")
    cppgoalregiontext = os.path.join(cppenvfolder, f"remind_{scannum}_goal_regions.txt")
    pypairfilename = os.path.join(pythonenvfolder, f"ReMIND_starts_{scannum}.txt")
    cpppairfilename = os.path.join(cppenvfolder, f"remind_{scannum}_sg_pairs.txt")
    pytorquefilename=os.path.join(pythonenvfolder,"torque_curvature.npy")
    cpptorquefilename = os.path.join(cppenvfolder, f"torque_curvature.txt")


    scan = nib.load(scanfilename)
    transform = scan.affine

    print(transform)
    scan_data = scan.get_fdata()
    scandims = np.shape(scan_data)

    inds = np.unique(scan_data)    
    starts = np.where(scan_data == _START)

    torque = np.load(pytorquefilename)

    if np.shape(starts)[1] > 0:
        start_index = np.random.randint(0,np.shape(starts)[1])
        start = np.array([starts[0][start_index], starts[1][start_index], starts[2][start_index]])

    xstart = start[0]
    ystart = start[1]
    zstart = start[2]

    if goal is None:
        if np.max(inds) >= _GOAL:
            goals = np.where(scan_data == _GOAL)
            goal_index = np.random.randint(0,np.shape(goals)[1])
            goal = np.array([goals[0][goal_index], goals[1][goal_index], goals[2][goal_index]])
        else:
            goal = np.average(np.where(scan_data == _TUMOR), axis=1)
    center = np.average(np.where(scan_data == _BRAIN), axis=1)


    xgoal = np.floor(goal[0]).astype(int)
    ygoal = np.floor(goal[1]).astype(int)
    zgoal = np.floor(goal[2]).astype(int)

    x = xstart
    y = ystart
    z = zstart

    print(f"goal: {goal} \t center: {center} \t start: {start} \t x: {x}   y: {y}   z: {z}")

    invtrans = np.linalg.inv(transform)

    # plot_slices(scan_data, x, y, z, mask=False)

    write_obstacle_files(pyobstaclefilename, scan_data, segmentationfilename, cppobstaclefilename, transform, scandims)


    # plot_slices(obstacles, x, y, z, mask=True)  

    write_segmentation_files(pyskullsegmentationfilename, scan_data, transform, cppskullsegmentationfilename, scandims)



    # plt.show()
    

    write_python_files(pypairfilename, pytextfilename, xstart, ystart, zstart, scandims, transform, xgoal, ygoal, zgoal, pyobstaclefilename, pyskullsegmentationfilename, torque, pytorquefilename)



    if os.path.exists(pypairfilename):
        sg_pairs = np.loadtxt(pypairfilename)
        if len(sg_pairs) > 0:
            xstart = sg_pairs[0,0]
            ystart = sg_pairs[0,1]
            zstart = sg_pairs[0,2]
            xgoal = sg_pairs[0,3]
            ygoal = sg_pairs[0,4]
            zgoal = sg_pairs[0,5]

            with open(cppstartgoaltext, "w+") as textfile:
                start = transform_xyz(transform, xstart, ystart, zstart)
                goal = transform_xyz(transform, xgoal, ygoal, zgoal)
                r = R.from_matrix(start[0:3,0:3])
                q = r.as_quat()
                print(q)
                lines = [f"{start[0,3]} {start[1,3]} {start[2,3]} {q[0]} {q[1]} {q[2]} {q[3]}\n", 
                         f"{goal[0,3]} {goal[1,3]} {goal[2,3]} {q[0]} {q[1]} {q[2]} {q[3]}\n"]
                textfile.writelines(lines)

            with open(cpptorquefilename, "w+") as textfile:
                lines = [f"{torque[0]} {torque[1]}\n"]

                textfile.writelines(lines)

            with open(cppgoalregiontext, "w+") as textfile:
                lines = [f"{torque[0]} {torque[1]}\n"]

                textfile.writelines(lines)           
            
            lines = []
            with open(cpppairfilename, "w+") as textfile:
                for i in range(np.shape(sg_pairs)[0]):
                    start = transform_xyz(transform, sg_pairs[i,0], sg_pairs[i,1], sg_pairs[i,2])
                    goal = transform_xyz(transform, sg_pairs[i,3], sg_pairs[i,4], sg_pairs[i,5])
                    lines += [f"{start[0,3]} {start[1,3]} {start[2,3]} {goal[0,3]} {goal[1,3]} {goal[2,3]}\n"]

                textfile.writelines(lines)

            lines = []
            with open(cppgoalregiontext, "w+") as textfile:
                inds = np.where(np.logical_and(sg_pairs[:,0] == xstart,np.logical_and(sg_pairs[:,1] == ystart, sg_pairs[:,2] == zstart)))[0]
                for i in range(np.shape(inds)[0]):
                    goal = transform_xyz(transform, sg_pairs[i,3], sg_pairs[i,4], sg_pairs[i,5])
                    lines += [f"{goal[0,3]} {goal[1,3]} {goal[2,3]}\n"]                    


def write_obstacle_files(pyobstaclefilename, scan_data, segmentationfilename, cppobstaclefilename, transform, scandims):
    if os.path.exists(pyobstaclefilename):
        obstacles = np.load(pyobstaclefilename)
    else:
        obstacles = np.logical_or(np.logical_not(scan_data == _BRAIN), scan_data == _VENTRICLES)
        obstacles = np.logical_and(np.logical_not(scan_data == _START), obstacles)
        obstacles = np.logical_and(np.logical_not(scan_data == _GOAL), obstacles)
        obstacles = np.logical_and(np.logical_not(scan_data == _TUMOR), obstacles).astype(int)
        np.save(pyobstaclefilename, obstacles)
        np.save(segmentationfilename, scan_data)

        obstacles_ = np.where(obstacles)
        obstaclepoints = np.empty(np.shape(obstacles_))
        if np.shape(obstacles_)[0] > 0:
            for i in range(np.shape(obstacles_)[1]):
                obstaclepoints[:,i] = np.array([obstacles_[0][i], obstacles_[1][i], obstacles_[2][i]])
            obstaclepoints = np.transpose(obstaclepoints)
            np.random.shuffle(obstaclepoints)

        f = open(cppobstaclefilename, 'a')
        np.savetxt(f, transform, fmt='%1.16f', newline="\n")
        np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
        np.savetxt(f, obstaclepoints, fmt='%d', delimiter=" ")


def write_segmentation_files(pyskullsegmentationfilename, scan_data, transform, cppskullsegmentationfilename, scandims):
    if os.path.exists(pyskullsegmentationfilename):
        skullpoints = np.loadtxt(pyskullsegmentationfilename, skiprows=5) # TODO: skip first 4 lines
    else:
        skull = np.where(scan_data == _SKULL)
        skullpoints = np.empty(np.shape(skull))
        if np.shape(skull)[0] > 0:
            for i in range(np.shape(skull)[1]):
                skullpoints[:,i] = np.array([skull[0][i], skull[1][i], skull[2][i]])
            skullpoints = np.transpose(skullpoints)
            np.random.shuffle(skullpoints)

        f = open(pyskullsegmentationfilename, 'a')
        np.savetxt(f, transform, fmt='%1.16f', newline="\n")
        np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
        np.savetxt(f, skullpoints, fmt='%d', delimiter=" ")

        f = open(cppskullsegmentationfilename, 'a')
        np.savetxt(f, transform, fmt='%1.16f', newline="\n")
        np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
        np.savetxt(f, skullpoints, fmt='%d', delimiter=" ")


def write_python_files(pypairfilename, pytextfilename, xstart, ystart, zstart, scandims, transform, xgoal, ygoal, zgoal, pyobstaclefilename, pyskullsegmentationfilename, torque, pytorquefilename):
    if os.path.exists(pypairfilename):
        sg_pairs = np.loadtxt(pypairfilename)
        if len(sg_pairs) > 0:
            np.random.shuffle(sg_pairs)
            np.savetxt(pypairfilename, sg_pairs, fmt='%d')


    with open(pytextfilename, "w+") as textfile:
        start = np.eye(4)
        start[0,3] = xstart
        start[1,3] = ystart
        start[2,3] = zstart

        lines = [f"Bounds: 0 {scandims[0]} 0 {scandims[1]} 0 {scandims[2]}\n", 
                f"Transform: {transform[0,0]} {transform[0,1]} {transform[0,2]} {transform[0,3]} {transform[1,0]} {transform[1,1]} {transform[1,2]} {transform[1,3]} {transform[2,0]} {transform[2,1]} {transform[2,2]} {transform[2,3]} {transform[3,0]} {transform[3,1]} {transform[3,2]} {transform[3,3]}\n",
                f"Start: {start[0,0]} {start[0,1]} {start[0,2]} {start[0,3]} {start[1,0]} {start[1,1]} {start[1,2]} {start[1,3]} {start[2,0]} {start[2,1]} {start[2,2]} {start[2,3]} {start[3,0]} {start[3,1]} {start[3,2]} {start[3,3]}\n", 
                f"Goal: {xgoal} {ygoal} {zgoal}\n", 
                f"Obstacles: {pyobstaclefilename}\n", 
                f"Skull: {pyskullsegmentationfilename}\n",
                f"Torque: {torque[0]} {torque[1]} {pytorquefilename}\n",
                f"Pairs: {pypairfilename}\n",
                f"NeedleRobot: 0 500 0 {_KAPPA} -3.14 3.14\n"]
        
        # min_k = get_min_curvature(lines, obstacles)
        # lines += [f"MinK: {min_k}\n"]

        if os.path.exists(pypairfilename):
            sg_pairs = np.loadtxt(pypairfilename)
            if len(sg_pairs) > 0:
                start[0,3] = sg_pairs[0,0]
                start[1,3] = sg_pairs[0,1]
                start[2,3] = sg_pairs[0,2]
                lines[2] = f"Start: {start[0,0]} {start[0,1]} {start[0,2]} {start[0,3]} {start[1,0]} {start[1,1]} {start[1,2]} {start[1,3]} {start[2,0]} {start[2,1]} {start[2,2]} {start[2,3]} {start[3,0]} {start[3,1]} {start[3,2]} {start[3,3]}\n"
                lines[3] = f"Goal: {sg_pairs[0,3]} {sg_pairs[0,4]} {sg_pairs[0,5]}\n"
        else:
            sg_pairs = verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals))
            np.savetxt(pypairfilename, sg_pairs, fmt='%d')
            if len(sg_pairs) == 0:
                print("no start/goal pairs found with non trivial solutions")

        textfile.writelines(lines)


def verify_ReMIND_env(lines, starts, goals):
    env = ReMINDEnvironment()
    env.read_lines(lines, variable_curvature=False)
    sg_pairs = []
    print(len(starts))
    print(len(goals))
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    for start in starts[0:200]:
        print(start)
        env.parse_start(['1.0', '0.0', '0.0', str(start[0]), '0.0', '1.0', '0.0', str(start[1]), '0.0', '0.0', '1.0', str(start[2]), '0.0', '0.0', '0.0', '1.0'])
        for goal in goals[0:200]:
            env.change_goal(goal)
            q, phi = env.robot.ik(env.goal)
            # print(f"start: {start} goal: {goal} q: {q} start_w: {env.start} goal_w: {env.goal}")
            if q is not None:
                env.goal 
                rrt = RRT(100, 3, 0.5, lims=env.lims, skull_tree=env.skulltree, r_curvature_line=env.torque, connect_prob=0.1, collision_func=env.test_collisions_world, custom_sample_func=env.sample_sphere_intersects_trumpet, variable_curvature=False)
                rrt.rrt_setup(env.robot, env.goal, phi_constraint=False)

                (status, new_node) = rrt.extend(rrt.T, env.goal, rejection=True)
                if not status == _REACHED:
                    sg_pairs += [[start[0], start[1], start[2], goal[0], goal[1], goal[2]]]
                # env.draw_path(None, rrt, dynamic_tree=False, dynamic_plan=False, show=True)

    return sg_pairs


def transform_xyz(transformation, x, y, z):
    mat = np.eye(4)
    mat[0,3] = x
    mat[1,3] = y
    mat[2,3] = z

    mat = np.matmul(transformation, mat)
    return mat


def get_min_curvature(lines, obstacles):
    env = ReMINDEnvironment()
    env.read_lines(lines, variable_curvature=True)
    sg_pairs = []
    obstacles = np.where(obstacles == False)
    obstacles = np.transpose(obstacles)
    min_k = env.robot.needle_lims[1,1]
    for start in obstacles:
        # print(start)
        env.parse_start(['1.0', '0.0', '0.0', str(start[0]), '0.0', '1.0', '0.0', str(start[1]), '0.0', '0.0', '1.0', str(start[2]), '0.0', '0.0', '0.0', '1.0'])
        new_lims = env.robot.get_lims(env.robot.gw)
        if new_lims[1,1] < min_k:
            min_k = new_lims[1,1]
    
    return min_k
        

def process_Pi_data(datafile, envfolder):
    """
    Processes data from Pi dataset, saving linear regression data to a .npy file
    
    Parameters:
        datafile (string): file path to pi data file
    """
    torquecurvaturefilename = os.path.join(envfolder, "torque_curvature.npy")
    mat = scipy.io.loadmat(datafile)
    readtorques = mat["torques"]
    radius = mat["kappa"]
    
    radius_by_stiffness = np.empty((3, radius.shape[1], radius.shape[2] - 1))
    
    radius_by_stiffness[0, :, :] = radius[4, :, 1:]  
    radius_by_stiffness[1, :, :] = radius[1, :, 1:]
    radius_by_stiffness[2, :, :] = radius[5, :, 1:]
    
    radius_by_stiffness_mean = np.squeeze(np.mean(radius_by_stiffness, 1));
    
    torques = np.array((readtorques[0,3], readtorques[0,2], readtorques[0,1]))
    radius_of_curvatures = np.array((radius_by_stiffness_mean[0,2], radius_by_stiffness_mean[0,1], radius_by_stiffness_mean[0,0]))

    bestFit = np.array(stats.linregress(torques, radius_of_curvatures))
    
    np.save(torquecurvaturefilename, bestFit) 


def plot_slices(scan_data, x, y, z, cmap="plasma", figname=None, mask=False):
    """
    Plots orthogonal slices of the segmented scan environment.

    Parameters:
    scan_data (n x m x p): segmented scan int array
    x (int): index for zy slice
    y (int): index for the zx slice
    z (int): index for the xy slice
    maxval (int): max value for the colormap
    cmap (string): colormap to use, default is plasma
    figname (string): name/save location for figure if desired, default is None which results in no saved figure
    mask (bool): True treats data as already masked, False (default) treats data as regular segmented data 

    """
    zy = scan_data[x,:,:]
    zx = scan_data[:,y,:]
    xy = scan_data[:,:,z]

    maxval = np.max(np.array([np.max(np.array(zy)), np.max(np.array(zx)), np.max(np.array(xy))]))

    plot_a_slice(xy, 6, cmap, figname, "xy", "z", z, mask)
    plot_a_slice(zy, 6, cmap, figname, "zy", "x", x, mask)
    plot_a_slice(zx, 6, cmap, figname, "zx", "y", y, mask)


def plot_a_slice(scan_slice, maxval, cmap, figname, slice_plane, slice_dir, slice_ind, mask):
    """
    Plots a single slice of the the segmented or masked brain.

    Parameters:
    scan_slice (a x b): data image
    maxval (int): max value for the colormap
    cmap (string): colormap to use when plotting
    figname (string): name/location to save image, if None does not save
    slice_plane (string): directions of the plane ordered such that [0] is along x and [1] is along y when plotted
    slice_dir (string): direction the slice was taken
    slice_ind (int): index of the slice taken
    mask (bool): True treat data as masked data, False treat as segmented data, impacts colorbar, labeling, and file name

    """
    plt.figure()


    plt.imshow(scan_slice, cmap=cmap, origin='lower', vmin=0, vmax=maxval)
    plt.xlabel(slice_plane[0])
    plt.ylabel(slice_plane[1])
    if mask:
        plt.title(f"Brain Mask {slice_plane} plane {slice_dir}-{slice_ind}")
        cbar = plt.colorbar()
        cbar.ax.set_yticks([0, 1, 4, 5])
        cbar.ax.set_yticklabels(['workspace', 'obstacles', 'start', 'goal'])

        if figname is not None:
            plt.savefig(f"{figname}-mask{slice_plane}.pdf")          
    else:
        plt.title(f"Brain Scan {slice_plane} plane {slice_dir}-{slice_ind}")
        cbar = plt.colorbar()
        cbar.ax.set_yticks([0, 1, 2, 3, 4, 5, 6])
        cbar.ax.set_yticklabels(['unoccupied', 'skull', 'brain', 'ventricles', 'tumor', 'start', 'goal'])

        if figname is not None:
            plt.savefig(f"{figname}-brain{slice_plane}.pdf")   


def create_test_env(r=150,spacing=150):
    """
    
    """
    width = np.round(2.2*r).astype(int)
    skull = np.zeros((width, width, width))
    center = width/2
    center_ = np.round(center).astype(int)
    thetas = np.linspace(0,2*np.pi,spacing)
    phis = np.linspace(0,2*np.pi,spacing)
    for theta in thetas:
        for phi in phis:
            vec = np.array([np.sin(phi)*np.cos(theta), np.sin(phi)*np.sin(theta), np.cos(phi)])
            vec = np.divide(vec, np.linalg.norm(vec))
            coord = np.round(np.add(np.multiply(vec,r),center)).astype(int)
            skull[coord[0], coord[1], coord[2]] = 1

    skull_coords = np.transpose(np.squeeze(np.where(skull == 1)))

    print(skull_coords)
    np.save("./ReMIND_envs/skull_segmentation_test.npy", skull_coords)


if __name__ == "__main__":
    # visualize_start_goal_positions()

    process_all_ReMIND("./../../data/ReMIND/", "./envs/", "./../data/input/")
    
    # process_Pi_data("./data/PiGroup/curvature_pi_group_data.mat", "./ReMIND_envs/")

    # create_test_env()

