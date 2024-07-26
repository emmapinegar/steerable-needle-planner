import sys
import open3d as o3d
import copy
import numpy as np
import matplotlib.pyplot as plt


colorBank = {
    "0": [1, 0.706, 0],
    "1": [0.3010, 0.7450, 0.9330],
    "2": [0.8500, 0.3250, 0.0980],
    "3": [0.4940, 0.1840, 0.5560],
    "4": [0.4660, 0.6740, 0.1880],
    "5": [0, 0.4470, 0.7410],
    "6": [0.6350, 0.0780, 0.1840],
    "7": [0.75, 0.75, 0],
    "8": [0.5, 0.5, 0.5],
    "size": 9
}


def plot_pc_o3d():
    fileNames = ["../data/input/goal_regions.txt", "../data/output/20240710-12-51-42_ptcloud.txt"] #, "../data/input/obstacles.txt"]

    obstacles_transform = np.array([[0.839844, 0, 0, -177.938],[0, 0.839844, 0, 70.0504],[0, 0, 1, -809.112],[0, 0, 0, 1]]).astype(np.float64)

    start_p = np.array([[-105.8324], [158.4900], [-610.0964]])
    start_q = np.array([[1], [0], [0], [0]])

    goal_p = np.array([[-105.8324], [158.4900], [-600.0964]])
    goal_q = np.array([[1], [0], [0], [0]])

    tau = 3

    start = o3d.geometry.TriangleMesh.create_coordinate_frame()
    world = copy.deepcopy(start)
    rot = start.get_rotation_matrix_from_quaternion(start_q)
    
    start.translate(start_p)
    start.rotate(rot)
    world.translate(start_p)
    


    goal = o3d.geometry.TriangleMesh.create_coordinate_frame()
    rot = goal.get_rotation_matrix_from_quaternion(goal_q)
    
    goal.translate(goal_p)
    goal.rotate(rot)

    curvature_lims = o3d.geometry.TriangleMesh.create_torus(torus_radius=20, tube_radius=19.5)
    curvature_lims.translate(start_p)

    olive_radius = np.linalg.norm(start_p - goal_p)/2 + 3
    print(olive_radius)
    olive_lims = o3d.geometry.TriangleMesh.create_sphere(radius=olive_radius)
    olive_lims.translate(start_p)
    olive_lims.paint_uniform_color(colorBank["7"])
    curvature_lims = o3d.t.geometry.TriangleMesh.from_legacy(curvature_lims)
    olive_lims = o3d.t.geometry.TriangleMesh.from_legacy(olive_lims)
    intersection = olive_lims.boolean_union(curvature_lims)
    o3d.visualization.draw([{'name': 'difference', 'geometry': intersection}])


    # ptcs = [world, start, goal, curvature_lims, olive_lims]
    # for i in range(len(fileNames)):
    #     ptcFile = fileNames[i]
    #     print(ptcFile)
    #     ptc = o3d.io.read_point_cloud(ptcFile, format='xyz')
    #     numpoints = np.shape(ptc.points)
    #     print(numpoints)
    #     if numpoints[0] > 100000:
    #         ptc = ptc.random_down_sample(0.02)
    #         ptc.transform(obstacles_transform)

    #     print("Point cloud {}: ".format(i))
    #     print(ptc)
        
    #     print("Drawing point cloud...")
    #     print(colorBank[str(i)])
    #     ptc.paint_uniform_color(colorBank[str(i)])
    #     ptcs.append(ptc)

    # o3d.visualization.draw_geometries(ptcs)    



def checkWorkspaceConnected(sp, sq, gp, gq, tau, r_min, el_max, voxel_rad=1):
    sg = gp - sp
    d = np.linalg.norm(sg)
    sg_hat = sg/d

    if (2*r_min - tau < d):
        max_h = d + tau
    else:
        r = r_min - tau
        cos_theta = (d*d + r_min*r_min - r*r)/(2*d*r_min)
        max_h = 2*r_min*cos_theta

    center_y = 0.5*max_h

    if r_min > center_y:
        center_x = np.sqrt(r_min*r_min - center_y*center_y)
    else:
        center_x = 0

    rugby_center = np.array([-center_x, center_y])
    rugby_rad = np.max([r_min, center_y])

    rugby_phi = 2*np.arcsin((2*rugby_rad + tau)/(rugby_rad))
    rugby_el = rugby_rad * rugby_phi

    plt.figure()
    plt.scatter(sp[0], sp[1], sp[2])
    plt.scatter(gp[0], gp[1], gp[2])


    




if __name__=='__main__':
    
    tau = 3
    el_max = 150
    r_min = 20
    phi_max = np.pi/2

    fileNames = ["../data/input/goal_regions.txt", "../data/input/start_and_goal_poses.txt", "../data/output/20240726-15-11-38_interp.txt", "../data/output/20240726-15-11-52_interp.txt", "../data/output/20240726-15-12-00_interp.txt", "../data/output/20240726-15-12-34_interp.txt", "../data/output/20240726-15-33-59_interp.txt"]
    # fileNames = ["../data/input/goal_regions.txt", "../data/output/20240712-13-31-42_interp.txt", "../data/output/20240712-13-35-44_interp.txt"]
    obstacles_transform = np.array([[0.839844, 0, 0, -177.938],[0, 0.839844, 0, 70.0504],[0, 0, 1, -809.112],[0, 0, 0, 1]]).astype(np.float64)

    start_p = np.array([[-105.8713], [158.2018], [-613.2506]])
    start_q = np.array([[-0.368893], [0], [-0.914127], [0.168198]]) # np.array([[1], [0], [0], [0]]) # w, x, y, z

    goal_p = np.array([[-54.8324], [139.4900], [-644.0964]])
    goal_q = np.array([[-0.385233], [0], [-0.920491], [-0.0655092]])


    start = o3d.geometry.TriangleMesh.create_coordinate_frame()
    world = copy.deepcopy(start)
    rot_s = start.get_rotation_matrix_from_quaternion(start_q)
    
    start.translate(start_p)
    start.rotate(rot_s)
    world.translate(start_p)
    


    goal = o3d.geometry.TriangleMesh.create_coordinate_frame()
    rot = goal.get_rotation_matrix_from_quaternion(goal_q)
    
    goal.translate(goal_p)
    goal.rotate(rot)

    curvature_lims = o3d.geometry.TriangleMesh.create_torus(torus_radius=50, tube_radius=49)
    curvature_lims.translate(start_p)
    curvature_lims.rotate(rot_s)

    olive_radius = np.max([50,np.linalg.norm(start_p - goal_p)/2 + 3])
    print(olive_radius)
    olive_lims = o3d.geometry.TriangleMesh.create_sphere(radius=olive_radius)
    olive_lims.translate(start_p + (goal_p - start_p)/2)
    olive_lims.paint_uniform_color(colorBank["7"])
    # curvature_lims = o3d.t.geometry.TriangleMesh.from_legacy(curvature_lims)
    # olive_lims = o3d.t.geometry.TriangleMesh.from_legacy(olive_lims)
    # intersection = olive_lims.boolean_union(curvature_lims)
    # o3d.visualization.draw([{'name': 'difference', 'geometry': intersection}])


    ptcs = [world, start, goal] #, curvature_lims, olive_lims]
    for i in range(len(fileNames)):
        ptcFile = fileNames[i]
        print(ptcFile)
        ptc = o3d.io.read_point_cloud(ptcFile, format='xyz')
        numpoints = np.shape(ptc.points)
        print(numpoints)
        if numpoints[0] > 100000:
            ptc = ptc.random_down_sample(0.02)
            ptc.transform(obstacles_transform)

        print("Point cloud {}: ".format(i))
        print(ptc)
        
        print("Drawing point cloud...")
        print(colorBank[str(i)])
        ptc.paint_uniform_color(colorBank[str(i)])
        ptcs.append(ptc)

    o3d.visualization.draw_geometries(ptcs) 