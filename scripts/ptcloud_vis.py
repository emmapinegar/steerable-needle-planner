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
        fileNames = ["../data/input/goal_regions.txt", "../data/input/remind_start_and_goal_poses.txt", "../data/input/remind_obstacles_outline_viz.txt", "../data/output/20241010-14-58-01_interp.txt"] #, "../data/output/20240930-12-52-15_interp.txt"]
    else:
        fileNames = sys.argv[1:]

    obstacles_transform = np.array([[0.2257, 0.1947, 0.0344, -83.7135],[0.1957, -0.2274, 0.0033, 106.4279],[0.0282, 0.0199, -0.2978, 43.7868],[0, 0, 0, 1]]).astype(np.float64)

    ptcs = []
    for i in range(len(fileNames)):
        ptcFile = fileNames[i]
        ptc = o3d.io.read_point_cloud(ptcFile, format='xyz')
        numpoints = np.shape(ptc.points)
        print(numpoints)
        if ptcFile.__contains__("obstacle"):
            ptc.transform(obstacles_transform)

        if numpoints[0] > 100000:
            ptc = ptc.random_down_sample(0.02)

        print("Point cloud {}: ".format(i))
        print(ptc)
        ptcs.append(ptc)

    draw_ptc(ptcs)