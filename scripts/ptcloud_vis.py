# BSD 3-Clause License

# Copyright (c) 2021, The University of North Carolina at Chapel Hill
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#! @author Mengyu Fu

import sys
import open3d as o3d
import copy
import numpy as np

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

def draw_ptc(ptc):
    ptcs = []

    for i in range(len(ptc)):
        ptcTemp = copy.deepcopy(ptc[i])
        colorIdx = i % colorBank["size"]
        ptcTemp.paint_uniform_color(colorBank[str(colorIdx)])
        ptcs.append(ptcTemp)

    o3d.visualization.draw_geometries(ptcs)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        # fileNames = ["../data/input/goal_regions.txt", "../data/input/start_and_goal_poses.txt", "../data/input/obstacles.txt", "../data/output/20240925-12-25-03_ptcloud.txt", "../data/output/20240925-12-25-03_interp.txt", "../data/output/20240925-12-24-44_ptcloud.txt", "../data/output/20240925-12-24-44_interp.txt", "../data/output/20240925-12-28-14_interp.txt", "../data/output/20240925-12-33-06_interp.txt"]
        fileNames = ["../data/input/remind_001_skull_outline_shuffled.txt", "../data/output/20250905-08-57-34_rrt_remind_001_org.txt", "../data/output/20250905-09-01-36_aorrt_remind_001_org.txt", "../data/output/20250905-09-05-40_rcs_remind_001_org.txt", "../data/output/20250905-09-09-01_rcs_star_remind_001_org.txt"] #, "../data/output/20250508-15-10-26_rrt_remind_003_interp.txt"]
    else:
        fileNames = sys.argv[1:]

    start_p = np.array([[0], [0], [0]])
    start_q = np.array([[0], [0], [1], [0]]) # np.array([[-0.0007], [0.0008], [0.0077], [0.9999]]) # w, x, y, z

    goal_p = np.array([[0], [0], [0]])
    goal_q = np.array([[0], [0], [1], [0]])


    start = o3d.geometry.TriangleMesh.create_coordinate_frame()
    world = copy.deepcopy(start)




    ptcs = [world]
    for i in range(len(fileNames)):
        ptcFile = fileNames[i]
        ptc = o3d.io.read_point_cloud(ptcFile, format='xyz')

        print(np.asarray(ptc.points)[:5,:])
        if ptcFile.__contains__("obstacle"):
            transform = np.loadtxt(ptcFile, max_rows=4)
            print(transform)
            ptc.transform(transform)
            ptc = ptc.random_down_sample(0.01)
        elif ptcFile.__contains__("skull"):
            transform = np.loadtxt(ptcFile, max_rows=4)
            print(transform)
            ptc.transform(transform)
            ptc = ptc.random_down_sample(0.1) 
        elif ptcFile.__contains__("org") or ptcFile.__contains__("interp"):
            path_points = np.loadtxt(ptcFile)
            start_p = path_points[0,0:3]
            start_q = path_points[0,3:7]
            goal_p = path_points[-1,0:3]
            goal_q = path_points[-1,3:7]
        elif ptcFile.__contains__("ptcloud"):
            path_points = np.loadtxt(ptcFile, max_rows=2)
            start_p = path_points[0,0:3]  
        elif ptcFile.__contains__("sg_pairs"):
            path_points = np.loadtxt(ptcFile, max_rows=2)     

        numpoints = np.shape(ptc.points)
        print(numpoints)

        if numpoints[0] > 10000000:
            ptc = ptc.random_down_sample(0.001)
        elif numpoints[0] > 1000000:
            ptc = ptc.random_down_sample(0.01)

        print("Point cloud {}: ".format(i))
        print(ptc)
        ptcs.append(ptc)


    rot_s = start.get_rotation_matrix_from_quaternion(start_q)
    
    start.translate(start_p)
    start.rotate(rot_s)
    world.translate(start_p)
    
    ptcs.append(start)

    goal = o3d.geometry.TriangleMesh.create_coordinate_frame()
    rot = goal.get_rotation_matrix_from_quaternion(goal_q)
    
    goal.translate(goal_p)
    goal.rotate(rot)
    
    ptcs.append(goal)

    draw_ptc(ptcs)