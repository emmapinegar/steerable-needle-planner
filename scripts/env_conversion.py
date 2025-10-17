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
        process_ReMIND_scan(scanfilename, pythonenvfolder, cppenvfolder, scannum)


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


    xgoal = np.floor(goal[0]).astype(int)
    ygoal = np.floor(goal[1]).astype(int)
    zgoal = np.floor(goal[2]).astype(int)

    x = xstart
    y = ystart
    z = zstart

    start = np.zeros((4,4))
    start[0,1] = -1
    start[1,0] = 1
    start[2,2] = 1
    start[3,3] = 1
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
            f"NeedleRobot: 0 100 0 {_KAPPA} -3.14 3.14\n"]

    # sg_pairs = verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=100)


    # pair_lines = []
    # with open(cpppairfilename, "a+") as textfile:
    #     for i in range(np.shape(sg_pairs)[0]):
    #         # print(np.shape(sg_pairs))
    #         # print(i)
    #         # print(sg_pairs[i][0])
    #         start = transform_xyz(transform, sg_pairs[i][0], sg_pairs[i][1], sg_pairs[i][2])
    #         goal = transform_xyz(transform, sg_pairs[i][3], sg_pairs[i][4], sg_pairs[i][5])
    #         pair_lines += [f"{start[0,3]} {start[1,3]} {start[2,3]} {goal[0,3]} {goal[1,3]} {goal[2,3]}\n"]

    #     textfile.writelines(pair_lines)

    sg_pairs = verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=250)

    pair_lines = []
    with open(cpppairfilename, "a+") as textfile:
        for i in range(np.shape(sg_pairs)[0]):
            start = transform_xyz(transform, sg_pairs[i][0], sg_pairs[i][1], sg_pairs[i][2])
            goal = transform_xyz(transform, sg_pairs[i][3], sg_pairs[i][4], sg_pairs[i][5])
            pair_lines += [f"{start[0,3]} {start[1,3]} {start[2,3]} {goal[0,3]} {goal[1,3]} {goal[2,3]}\n"]

        textfile.writelines(pair_lines)


    # print(f"goal: {goal} \t center: {center} \t start: {start} \t x: {x}   y: {y}   z: {z}")

    # plot_slices(scan_data, x, y, z, mask=False)

    # write_obstacle_files(pyobstaclefilename, scan_data, segmentationfilename, cppobstaclefilename, transform, scandims)


    # # plot_slices(obstacles, x, y, z, mask=True)  

    # write_segmentation_files(pyskullsegmentationfilename, scan_data, transform, cppskullsegmentationfilename, scandims)



    # # plt.show()
    

    # write_python_files(pypairfilename, pytextfilename, xstart, ystart, zstart, scandims, transform, xgoal, ygoal, zgoal, pyobstaclefilename, pyskullsegmentationfilename, torque, pytorquefilename, starts, goals)



    # if os.path.exists(pypairfilename):
    #     sg_pairs = np.loadtxt(pypairfilename)
    #     if len(sg_pairs) > 0:

    #         xstart = sg_pairs[0,0]
    #         ystart = sg_pairs[0,1]
    #         zstart = sg_pairs[0,2]
    #         xgoal = sg_pairs[0,3]
    #         ygoal = sg_pairs[0,4]
    #         zgoal = sg_pairs[0,5]

    #         with open(cppstartgoaltext, "w+") as textfile:
    #             start = transform_xyz(transform, xstart, ystart, zstart)
    #             goal = transform_xyz(transform, xgoal, ygoal, zgoal)

    #             r = R.from_quat([0,0, 1, 0], scalar_first=False)
    #             # print(r.as_matrix())
    #             # start_t = np.zeros((3,3))
    #             # start_t[1,0] = 1
    #             # start_t[0,1] = 1
    #             # start_t[2,2] = -1
    #             # r = R.from_matrix(start_t)
    #             q = r.as_quat()
    #             # print(q)
    #             lines = [f"{start[0,3]} {start[1,3]} {start[2,3]} {q[0]} {q[1]} {q[2]} {q[3]}\n", 
    #                      f"{goal[0,3]} {goal[1,3]} {goal[2,3]} {q[0]} {q[1]} {q[2]} {q[3]}\n"]
    #             textfile.writelines(lines)

    #         with open(cpptorquefilename, "w+") as textfile:
    #             lines = [f"{torque[0]} {torque[1]}\n"]

    #             textfile.writelines(lines)

    #         with open(cppgoalregiontext, "w+") as textfile:
    #             lines = [f"{torque[0]} {torque[1]}\n"]

    #             textfile.writelines(lines)           
            
    #         lines = []
    #         with open(cpppairfilename, "w+") as textfile:
    #             for i in range(np.shape(sg_pairs)[0]):
    #                 start = transform_xyz(transform, sg_pairs[i,0], sg_pairs[i,1], sg_pairs[i,2])
    #                 goal = transform_xyz(transform, sg_pairs[i,3], sg_pairs[i,4], sg_pairs[i,5])
    #                 lines += [f"{start[0,3]} {start[1,3]} {start[2,3]} {goal[0,3]} {goal[1,3]} {goal[2,3]}\n"]

    #             textfile.writelines(lines)

    #         lines = []
    #         with open(cppgoalregiontext, "w+") as textfile:
    #             inds = np.where(np.logical_and(sg_pairs[:,0] == xstart,np.logical_and(sg_pairs[:,1] == ystart, sg_pairs[:,2] == zstart)))[0]
    #             for i in range(np.shape(inds)[0]):
    #                 goal = transform_xyz(transform, sg_pairs[i,3], sg_pairs[i,4], sg_pairs[i,5])
    #                 lines += [f"{goal[0,3]} {goal[1,3]} {goal[2,3]}\n"]

    #             textfile.writelines(lines)                    


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


def write_python_files(pypairfilename, pytextfilename, xstart, ystart, zstart, scandims, transform, xgoal, ygoal, zgoal, pyobstaclefilename, pyskullsegmentationfilename, torque, pytorquefilename, starts, goals):
    if os.path.exists(pypairfilename):
        sg_pairs = np.loadtxt(pypairfilename)
        # if len(sg_pairs) > 0:
        #     np.random.shuffle(sg_pairs)
        #     np.savetxt(pypairfilename, sg_pairs, fmt='%d')


    with open(pytextfilename, "w+") as textfile:
        start = np.zeros((4,4))
        start[0,1] = -1
        start[1,0] = 1
        start[2,2] = 1
        start[3,3] = 1
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
                f"NeedleRobot: 0 100 0 {_KAPPA} -3.14 3.14\n"]
        
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
            # sg_pairs = verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=15)
            # sg_pairs += verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=25)
            # sg_pairs += verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=50)
            sg_pairs += verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=100)
            sg_pairs += verify_ReMIND_env(lines, np.transpose(starts), np.transpose(goals), k=250)
            # np.random.shuffle(sg_pairs)
            np.savetxt(pypairfilename, sg_pairs, fmt='%d')
            if len(sg_pairs) == 0:
                print("no start/goal pairs found with non trivial solutions")

        textfile.writelines(lines)


def verify_ReMIND_env(lines, starts, goals, k=15):
    env = ReMINDEnvironment()
    env.read_lines(lines, variable_curvature=False)
    sg_pairs = []
    print(len(starts))
    print(len(goals))
    np.random.shuffle(starts)
    np.random.shuffle(goals)
    num_pairs = 1000
    num_goals = 20
    # if k > 100:
    #     num_pairs = 7500
    # elif k > 50:
    #     num_pairs = 5000
    
    open_goals = []
    for goal in goals:
        env.change_goal(goal)
        if not env.test_collisions_buffered(env.goal):
            open_goals += [goal]

    print(f"goals: {np.shape(goals)} good: {len(open_goals)}")
    env.robot.needle_lims[1,1] = 1/k
    i = 0
    for start in starts:
        # print(start)
        env.parse_start(['1.0', '0.0', '0.0', str(start[0]), '0.0', '1.0', '0.0', str(start[1]), '0.0', '0.0', '1.0', str(start[2]), '0.0', '0.0', '0.0', '1.0'])
        if not env.test_collisions_buffered(env.start):
            next_pairs = []
            np.random.shuffle(open_goals)
            for goal in open_goals:
                if i < num_goals and len(sg_pairs) < num_pairs:
                    env.change_goal(goal)
                    if not env.test_collisions_world(env.goal):
                        valid = True
                        q, phi = env.robot.ik(env.goal)
                        # print(f"start: {start} goal: {goal} q: {q} start_w: {env.start} goal_w: {env.goal}")
                        if q is not None:
                            rrt = RRT(100, 3, 0.5, lims=env.lims, skull_tree=env.skulltree, r_curvature_line=env.torque, connect_prob=0.1, collision_func=env.test_collisions_world, custom_sample_func=env.sample_sphere_intersects_trumpet, variable_curvature=False)
                            rrt.rrt_setup(env.robot, env.goal, phi_constraint=False)
                            (status, new_node) = rrt.extend(rrt.T, env.goal, parent=env.start, k=0)

                            if status == _REACHED:
                                valid = False
                            # env.draw_path(None, rrt, dynamic_tree=False, dynamic_plan=False, show=True)
                        else:
                            valid = False

                        if valid:
                            next_pairs += [[start[0], start[1], start[2], goal[0], goal[1], goal[2]]]
                            i += 1
            if i == num_goals:
                sg_pairs += next_pairs
                print(len(sg_pairs))
        
        i = 0

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
        

def process_Pi_data(datafile, torquefilename, cpptorquefilename):
    """
    Processes data from Pi dataset, saving linear regression data to a .npy file
    
    Parameters:
        datafile (string): file path to pi data file
    """
    if not os.path.exists(torquefilename):
        mat = scipy.io.loadmat(datafile)
        readtorques = mat["torques"]
        radius = mat["kappa"]
        
        radius_by_stiffness = np.empty((3, radius.shape[1], radius.shape[2] - 1))
        
        radius_by_stiffness[0, :, :] = radius[4, :, 1:]  
        radius_by_stiffness[1, :, :] = radius[1, :, 1:]
        radius_by_stiffness[2, :, :] = radius[5, :, 1:]
        
        radius_by_stiffness_mean = np.squeeze(np.mean(radius_by_stiffness, 1))
        
        torques = np.array((readtorques[0,3], readtorques[0,2], readtorques[0,1], 0))
        radius_of_curvatures = np.array((radius_by_stiffness_mean[0,2], radius_by_stiffness_mean[0,1], radius_by_stiffness_mean[0,0], 0))

        bestFit = np.array(stats.linregress(torques, radius_of_curvatures))
        
        np.save(torquefilename, bestFit) 
        with open(cpptorquefilename, "w+") as textfile:
            lines = [f"{bestFit[0]} {bestFit[1]}\n"]

            textfile.writelines(lines)


    else:
        mat = scipy.io.loadmat(datafile)
        readtorques = mat["torques"]
        radius = mat["kappa"]
        
        radius_by_stiffness = np.empty((3, radius.shape[1], radius.shape[2] - 1))
        
        radius_by_stiffness[0, :, :] = radius[4, :, 1:]  # this is the brain stiffness data
        radius_by_stiffness[1, :, :] = radius[1, :, 1:]
        radius_by_stiffness[2, :, :] = radius[5, :, 1:]

        
        radius_by_stiffness_mean = np.squeeze(np.mean(radius_by_stiffness, 1))
        torques = np.array((readtorques[0,3], readtorques[0,2], readtorques[0,1], 0))
        radius_of_curvatures = np.array((radius_by_stiffness_mean[0,2], radius_by_stiffness_mean[0,1], radius_by_stiffness_mean[0,0], 0)) # only use the brain stiffness data

        bestFit = np.array(stats.linregress(torques, radius_of_curvatures))
        m = bestFit[0]
        b = bestFit[1]
        torques_ = torques[:3]
        print(torques)
        print(radius_by_stiffness[0,0,:])
        print(radius_by_stiffness_mean)
        # print(radius)
        # print(mat)
        plt.figure()
        plt.scatter(torques_, 1000/np.array([radius_by_stiffness[0,0,2],radius_by_stiffness[0,0,1],radius_by_stiffness[0,0,0]]), c='k', marker='o', s=120, alpha=0.1)
        plt.scatter(torques_, 1000/np.array([radius_by_stiffness[0,1,2],radius_by_stiffness[0,1,1],radius_by_stiffness[0,1,0]]), c='k', marker='o', s=120, alpha=0.1)
        plt.scatter(torques_, 1000/np.array([radius_by_stiffness[0,2,2],radius_by_stiffness[0,2,1],radius_by_stiffness[0,2,0]]), c='k', marker='o', s=120, alpha=0.1)
        plt.scatter(torques_, 1000/np.array([radius_by_stiffness[0,3,2],radius_by_stiffness[0,3,1],radius_by_stiffness[0,3,0]]), c='k', marker='o', s=120, alpha=0.1)
        plt.scatter(torques_, 1000/np.array([radius_by_stiffness[0,4,2],radius_by_stiffness[0,4,1],radius_by_stiffness[0,4,0]]), c='k', marker='o', s=120, alpha=0.1)

        plt.scatter(torques[:3], 1000/radius_of_curvatures[:3], s=30, alpha=1)
        plt.scatter(torques_, 1000/(torques_*m + b), s=30, alpha=1)
        extents = np.array([3e-5, 3.5e-5, 4e-5, 5e-5, 6e-5, 7e-5, 8e-5, torques_[2], torques_[1], torques_[0]])
        plt.plot(extents, 1000/(extents*m + b), c='k', linestyle='--')

        print(1000/(extents*m + b))
        print(bestFit)

        # https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.ticklabel_format.html
        plt.ticklabel_format(axis='x', style='sci', scilimits=(-3,3))
        plt.xlabel("Torque (Nm)")
        plt.ylabel("Radius of Curvature (mm)")
        plt.show()


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

    cpptorquefilename = os.path.join("./../data/input/", f"torque_curvature.txt")
    # process_Pi_data("./../../data/PiGroup/curvature_pi_group_data.mat", "./envs/torque_curvature.npy", cpptorquefilename)

    process_all_ReMIND("./../../data/ReMIND/", "./envs/", "./../data/input/")


