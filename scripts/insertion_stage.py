"""
Env_Creator: Provided some medical scans, create a .txt representation of the environment for use in collisions.py
University of Utah
April 2023
"""
import os
import shutil
# import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R


from needle import SteerableNeedle
from environment import ReMINDEnvironment
from rrt import RRT, _REACHED

import scipy.io
import nibabel as nib
from scipy import stats

_KAPPA = 0.04
_OBSTACLES = 7
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
    filenames.sort()
    segmentations.sort()
    print(filenames)
    digits = [f.strip("ReMIND-sgmntaiond./") for f in filenames]
    print(digits)
    np.random.seed(43829472)
    numfiles = len(filenames)

    if not os.path.exists(pythonenvfolder):
        os.makedirs(pythonenvfolder)

    for i in range(numfiles):
        scanfilename = segmentations[i]
        scannum = digits[i]

        # if not os.path.exists(obstaclefilename+".npy") or not os.path.exists(segmentationfilename) or not os.path.exists(skullsegmentationfilename) or not os.path.exists(textfilename) or not os.path.exists(startfilename):
        print(f"\nprocessing scan {scannum}")
        if i < 1:
            write_insertion_stage_file(scanfilename)
        # process_ReMIND_scan(scanfilename, pythonenvfolder, cppenvfolder, scannum)


def process_ReMIND_scan(scanfilename, pythonenvfolder, cppenvfolder, scannum, goal=None):
    """
    Processes the segmentation of a single scan from the ReMIND dataset.

    Parameters:
        scanfilename (str): name of the .nii segmentation file to process
        pythonenvfolder (str): file path to save the python environment files to
        cppenvfolder (str): file path to save the cpp environment files to
        scannum (str): the index of the scan with leading zeros to have 3 digits
    """
    pyobstaclefilename = os.path.join(pythonenvfolder, f"ReMIND_obstacles_{scannum}.npy")
    cppobstaclefilename = os.path.join(cppenvfolder, f"remind_{scannum}_obstacles.txt")
    segmentationfilename = os.path.join(pythonenvfolder, f"ReMIND_segmentation_{scannum}.npy")
    pyskullsegmentationfilename = os.path.join(pythonenvfolder, f"ReMIND_skull_{scannum}_outline_shuffled.txt")
    cppskullsegmentationfilename = os.path.join(cppenvfolder, f"remind_{scannum}_skull_outline_shuffled.txt")
    pytextfilename = os.path.join(pythonenvfolder, f"ReMIND_info_{scannum}.txt")
    cppstartgoaltext = os.path.join(cppenvfolder, f"remind_{scannum}_start_and_goal_poses.txt")
    cppgoalregiontext = os.path.join(cppenvfolder, f"remind_{scannum}_goal_regions.txt")
    pypairfilename = os.path.join(pythonenvfolder, f"ReMIND_starts_{scannum}_dynamic.txt")
    cpppairfilename = os.path.join(cppenvfolder, f"remind_{scannum}_sg_pairs_dynamic.txt")
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

    # if not os.path.exists(cpptorquefilename):
    #     with open(cpptorquefilename, "w+") as textfile:
    #         lines = [f"{torque[0]} {torque[1]}\n"]

    #         textfile.writelines(lines)        

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
    # center = np.average(np.where(scan_data == _BRAIN), axis=1)


def write_obstacle_files(pyobstaclefilename, scan_data, segmentationfilename, cppobstaclefilename, transform, scandims):
    if os.path.exists(pyobstaclefilename):
        obstacles = np.load(pyobstaclefilename)
        if not os.path.exists(cppobstaclefilename):
            obstacles = np.logical_or(scan_data == _OBSTACLES, scan_data == _VENTRICLES)
            obstacles_ = np.where(obstacles)
            obstaclepoints = np.empty(np.shape(obstacles_))
            if np.shape(obstacles_)[0] > 0:
                for i in range(np.shape(obstacles_)[1]):
                    obstaclepoints[:,i] = np.array([obstacles_[0][i], obstacles_[1][i], obstacles_[2][i]])
                obstaclepoints = np.transpose(obstaclepoints)
                np.random.shuffle(obstaclepoints)

            f = open(cppobstaclefilename, 'a')
            np.savetxt(f, transform, fmt='%1.20f', newline="\n")
            np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
            np.savetxt(f, obstaclepoints, fmt='%d', delimiter=" ")
    else:
        obstacles = np.logical_or(np.logical_not(scan_data == _BRAIN), scan_data == _VENTRICLES)
        obstacles = np.logical_and(np.logical_not(scan_data == _START), obstacles)
        obstacles = np.logical_and(np.logical_not(scan_data == _GOAL), obstacles)
        obstacles = np.logical_and(np.logical_not(scan_data == _TUMOR), obstacles).astype(int)
        np.save(pyobstaclefilename, obstacles)
        np.save(segmentationfilename, scan_data)

        obstacles = np.logical_or(scan_data == _OBSTACLES, scan_data == _VENTRICLES)
        obstacles_ = np.where(obstacles)
        obstaclepoints = np.empty(np.shape(obstacles_))
        if np.shape(obstacles_)[0] > 0:
            for i in range(np.shape(obstacles_)[1]):
                obstaclepoints[:,i] = np.array([obstacles_[0][i], obstacles_[1][i], obstacles_[2][i]])
            obstaclepoints = np.transpose(obstaclepoints)
            np.random.shuffle(obstaclepoints)

        f = open(cppobstaclefilename, 'a')
        np.savetxt(f, transform, fmt='%1.20f', newline="\n")
        np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
        np.savetxt(f, obstaclepoints, fmt='%d', delimiter=" ")


def write_segmentation_files(pyskullsegmentationfilename, scan_data, transform, cppskullsegmentationfilename, scandims):
    if os.path.exists(pyskullsegmentationfilename):
        skullpoints = np.loadtxt(pyskullsegmentationfilename, skiprows=5) # TODO: skip first 4 lines
        if not os.path.exists(cppskullsegmentationfilename):
            f = open(cppskullsegmentationfilename, 'a')
            np.savetxt(f, transform, fmt='%1.20f', newline="\n")
            np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
            np.savetxt(f, skullpoints, fmt='%d', delimiter=" ")           
    else:
        skull = np.where(scan_data == _SKULL)
        skullpoints = np.empty(np.shape(skull))
        if np.shape(skull)[0] > 0:
            for i in range(np.shape(skull)[1]):
                skullpoints[:,i] = np.array([skull[0][i], skull[1][i], skull[2][i]])
            skullpoints = np.transpose(skullpoints)
            np.random.shuffle(skullpoints)

        f = open(pyskullsegmentationfilename, 'a')
        np.savetxt(f, transform, fmt='%1.20f', newline="\n")
        np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
        np.savetxt(f, skullpoints, fmt='%d', delimiter=" ")

        f = open(cppskullsegmentationfilename, 'a')
        np.savetxt(f, transform, fmt='%1.20f', newline="\n")
        np.savetxt(f, np.array([scandims[0], scandims[1], scandims[2]]).reshape(1, -1), fmt='%d', delimiter=" ")
        np.savetxt(f, skullpoints, fmt='%d', delimiter=" ")


def write_insertion_stage_file(scanfilename):
    stage_points = np.loadtxt("./../data/input/insertion_stage_shuffled.txt", skiprows=5)
    stage_transform = np.loadtxt("./../data/input/insertion_stage_shuffled.txt", max_rows=4)
    scan = nib.load(scanfilename)
    transform = scan.affine

    print(transform)
    scan_data = scan.get_fdata()
    scandims = np.shape(scan_data)

    inverse_transform = np.linalg.inv(transform)
    stage_points = np.hstack((stage_points, np.ones((np.shape(stage_points)[0], 1))))
    stage_points = np.transpose(stage_points)
    stage_points = np.matmul(stage_transform, stage_points)
    stage_inds = np.matmul(inverse_transform, stage_points)
    stage_inds = np.vstack((np.array([scandims[0], scandims[1], scandims[2]]).reshape(1,-1), np.transpose(stage_inds[0:3,:])))
    print(stage_inds)
    stage_ind_transform = np.matmul(np.linalg.inv(stage_transform), transform)
    print(stage_ind_transform)
    # f = open("./../data/input/remind_001_insertion_stage_shuffled.txt", 'w')
    # np.savetxt(f, stage_ind_transform, fmt='%1.20f', newline="\n")
    np.savetxt("./../data/input/remind_001_insertion_stage_shuffled.txt", stage_inds, fmt='%d', delimiter=" ", header=f"{stage_ind_transform[0,0]:1.20f} {stage_ind_transform[0,1]:1.20f} {stage_ind_transform[0,2]:1.20f} {stage_ind_transform[0,3]:1.20f}\n{stage_ind_transform[1,0]:1.20f} {stage_ind_transform[1,1]:1.20f} {stage_ind_transform[1,2]:1.20f} {stage_ind_transform[1,3]:1.20f}\n{stage_ind_transform[2,0]:1.20f} {stage_ind_transform[2,1]:1.20f} {stage_ind_transform[2,2]:1.20f} {stage_ind_transform[2,3]:1.20f}\n{stage_ind_transform[3,0]:1.20f} {stage_ind_transform[3,1]:1.20f} {stage_ind_transform[3,2]:1.20f} {stage_ind_transform[3,3]:1.20f}")
    # np.savetxt(f, stage_inds, fmt='%d', delimiter=" ")



if __name__ == "__main__":


    process_all_ReMIND("./../../data/ReMIND/", "./envs/", "./../data/input/")


