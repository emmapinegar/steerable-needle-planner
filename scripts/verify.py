from rrt import RRT
from needle import SteerableNeedle
from environment import ReMINDEnvironment
import numpy as np
from scipy.spatial.transform import Rotation as R
import os
I = 0

def verify_var_curve(env_file:str, points_file:str, variable_curvature:bool=True):
    """
    Verifies the points recorded as part of the search tree are reachable.

    Parameters:
        env_file (str): file name and path for the needle environment text file
        points_file (str): file name and path for the text file containing the points in order of addition to the tree/path
        variable_curvature (bool): checks dynamic curvature constraints if true (default), checks fixed curvature constraint if false
    """
    pe = ReMINDEnvironment()
    pe.read_env(env_file, variable_curvature)
    

    points = np.loadtxt(points_file)
    # print(points)
    # print([points[0,3], points[0,4], points[0,5], points[0,6]])
    # r = R.from_quat([points[1,3], points[1,4], points[1,5], points[1,6]], scalar_first=True)
    # r = R.from_quat([0.707107, 0.707107, 0.0, 0.0], scalar_first=False)
    # print(r.as_matrix())


    # r_ = R.from_quat([0.0, 0.0178562, 0.0, 0.999841], scalar_first=False)
    # print(r_.as_matrix())

    # print(np.matmul(r.as_matrix(),r_.as_matrix()))
    samples = points[:,0:3]
    parents = points[:,3:6]
    gw = np.eye(4)
    

    gw[0,0] = -1
    gw[2,2] = -1
    gw[0,3] = samples[0,0]
    gw[1,3] = samples[0,1]
    gw[2,3] = samples[0,2]
    pe.change_gw(gw)
    num_samples = np.shape(samples)[0]
    step_length = 0.5
    connect_prob = 0.00

    # print(points)
    # print(np.shape(points))


    def sample_tree(i):
        # print(points)
        # print(f"sampled: {points[i,:]}")
        
        # print(i)
        return samples[i,:], parents[i,:]
    

    rrt = RRT(num_samples, len(pe.start), step_length, lims=pe.lims, skull_tree=pe.skulltree, r_curvature_line=pe.torque, connect_prob=connect_prob, collision_func=pe.test_collisions_world, custom_sample_func=sample_tree, variable_curvature=variable_curvature, time_limit=1000000)


    path, plan, phi, cost = rrt.rebuild_tree(pe.robot, pe.goal, phi_constraint=False)

    # pe.draw_path(None, rrt)


if __name__=='__main__':
    dir = "./../data/output/"
    # verify_var_curve("./envs/ReMIND_info_001.txt", dir + "20250825-11-16-58_rrt_remind_001_ptcloud.txt")


    files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("rcs_remind_001")]
    print(files)
    for file in files:
        print(f"\nprocessing {file}...")
        verify_var_curve("./envs/ReMIND_info_001.txt", dir + file)

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("003")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_003.txt", dir + file)

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("006")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_006.txt", dir + file)  

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("008")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_008.txt", dir + file)      

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("009")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_009.txt", dir + file)

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("010")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_010.txt", dir + file)

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("013")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_013.txt", dir + file)  

    # files = [file for file in os.listdir(path=dir) if file.__contains__("ptcloud.txt") and file.__contains__("015")]
    # print(files)
    # for file in files:
    #     print(f"\nprocessing {file}...")
    #     verify_var_curve("./envs/ReMIND_info_015.txt", dir + file)     