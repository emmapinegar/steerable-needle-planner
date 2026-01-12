from typing import Optional
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from math import cos, sin, atan2, pi, inf
from magnet import Magnet

from sklearn.neighbors import KDTree
_DEBUG = False

# Define relevent values for magnet calulations
_MANIPMAG_STRENGTH = 66.03
_SCREWMAG_STRENGTH = 0.0018
# MOVE THESE LATER TO BE CHANGABLE (or just somewhere else)
_MAXPHI = np.pi/2
_MAXK = 0.072



class SteerableNeedle:
    def __init__(self, needle_lims:npt.NDArray=None, p:npt.NDArray=None, gw:npt.NDArray=None, q:tuple[float,float,float]=None, phi:float=0.0, l:float=0.0, phi_constraint:bool=False, skull_tree:KDTree=None, r_curvature_line:npt.NDArray=np.array([0,0]), variable_curvature:bool=False):
        """
        Steerable needle object that can be used to calculate forward kinematics, inverse kinematics, test reachability of an action, etc. 

        Parameters:
            needle_lims (3x2 ndarray): [[arc min, arc max], [curvature min, curvature max], [theta min, theta max]]
            p (array): [x, y, z] location of the needle in world frame, overriden by location specified in gw
            gw (3x3 ndarray): transformation matrix of the needle from needle to world frame
            q (tuple[float, float , float]): [l (arc), k (curvature), theta (angle)] the control action used to get to the current needle location/pose
            skull_tree (kdtree): kdtree used to find the closest point in the skull
            r_curvature_line (array): array holding values for defining the curvature best fit line
        """
        if q is None:
            self.q = np.array([0, 0, 0])
        else:
            self.q = np.array(q)

        if p is None:
            self.p = np.array([10, 10, 1])
        else:
            self.p = np.array(p)

        if gw is None:
            self.gw = np.array([[1, 0, 0, self.p[0]], [0, 1, 0, self.p[1]], [0, 0, 1, self.p[2]], [0, 0, 0, 1]])
        else:
            self.gw = np.array(gw)
            self.p = np.array(gw[0:3,3])

        self.gw_inv = np.linalg.inv(self.gw)

        if needle_lims is None:
            self.needle_lims = np.array([[0, 250], [0, 0.2], [-pi, pi]])
        else:
            self.needle_lims = np.array(needle_lims)
        
        self.phi_constraint = phi_constraint
        self.phi = phi
        self.l = l
        self.skull_tree:KDTree = skull_tree
        self.r_curvature_line = r_curvature_line
        self.variable_curvature = variable_curvature

    def fk(self, q:tuple[float, float, float]) -> npt.NDArray:
        """
        Compute forward kinematics for the robot using action q.

        Parameters:
            q (tuple[float, float, float]): [l (arc), k (curvature), theta (angle)] the action to apply to move the current needle

        Returns:
            g_new (4x4 ndarray): a transformation matrix g_new from the resulting needle pose to world frame
        """
        l = q[0]
        k = q[1]
        theta = q[2]
        phi = l*k
        xm = np.array([-sin(theta), cos(theta), 0, 0])
        ym = np.array([-cos(theta)*cos(phi), -sin(theta)*cos(phi), sin(phi), 0])
        zm = np.array([cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi), 0])
        if k > 1e-10:
            dm = np.array([cos(theta)*(1 - cos(phi))/k, sin(theta)*(1 - cos(phi))/k, sin(phi)/k, 1])
        else:
            dm = np.array([0, 0, l, 1])

        gm = np.transpose(np.vstack((xm, ym, zm, dm)))
        g_new = np.matmul(self.gw, gm)
        return g_new

    def ik(self, p:npt.NDArray, print_:bool=False):
        """
        Compute inverse kinematics for the robot with position p.

        Parameters:
            p (array): [x, y, z] desired location of the needle in world frame

        Returns:
            q,phi (tuple[tuple[float, float, float] | None, float]): [l (arc), k (curvature), theta (angle)], phi the action used to move the current needle to the specified p
                            None if the desired p is not reachable
        """
        dp = np.matmul(self.gw_inv, np.transpose(np.append(p,1)))
        px = dp[0]
        py = dp[1]
        pz = dp[2]
        print_str = ""
        if _DEBUG or print_:
            print_str = print_str + f"sample: {np.round(p.reshape(3,),15)} parent: {np.round(self.p.reshape(3,),15)} dp: {np.round(dp[0:3],15)}"
        if not self.reachable(dp[0:3], print_=print_):
            if _DEBUG or print_: 
                print(f"not reachable {print_str} k: {self.needle_lims[1,1]}")
            return None, 0.0
        if np.linalg.norm(dp[0:3]) < 5e-10:
            return (0, 0, 0), 0

        theta = atan2(py,px) 
        xy_sq = np.linalg.norm(dp[0:2])

        k = (2*xy_sq)/(np.sum(np.square(dp[0:3])))


        if k == 0.0 or k < 1e-5:
            l = pz
            k = 0.0
            phi = 0.0
            theta = -np.pi/2
            r = inf
        else:
            r = (np.sum(np.square(dp[0:3])))/(2*xy_sq)
            k = 1/r
            phi = atan2(pz, r - xy_sq)
            if phi < 0:
                phi = 2*np.pi + phi
            l = phi*r

        # arc length limit
        if self.l + l > self.needle_lims[0,1]:
            if _DEBUG or print_:
                print(f"violates insertion length! {self.l + l} {print_str}")
            return None, 0.0

        # 90 degree constraint
        if self.phi_constraint and self.phi + phi > _MAXPHI:
            if _DEBUG or print_:
                print(f"phi rejected! {self.phi + phi} {print_str}" )
            return None, 0.0
        if _DEBUG or print_:
            print(f"{print_str} l: {round(l,4)} \tk: {round(k,10)} \ttheta: {round(theta,10)} \tr: {round(r,4)} \tphi: {round(phi,4)}")   
        q = (l, k, theta)
        return q, phi


    def get_distance(self, p:npt.NDArray) -> float:
        """
        Get the distance to point p based on the needles pose.

        Parameters:
            p (NDArray): point in world frame to get distance from needle to

        Returns:
            distance (float): arc length from needle's location to p
        """
        dp = np.matmul(self.gw_inv, np.transpose(np.append(p,1)))

        if np.linalg.norm(dp[0:3]) < 5e-10:
            return 0.0
        
        q, phi = self.ik(p)
        if q is None:
            distance = 10000
        else:
            distance = q[0]

        return distance     

   
    def reachable(self, p:npt.NDArray, print_=False, check_y=False) -> bool:
        """
        Tests if a point p is reachable by the current pose of the needle given the curvature limits.

        Parameters:
            p (array): [x, y, z] desired location of the needle in the current needle frame
            print_ (bool): debugging print flag if True, default is False
            check_y (bool): checks level of y if True and oldcheck is False, default is False
            oldcheck (bool): performs older published Rg RRT reachability check if True, default is False

        Returns:
            reach (bool): True if the point is reachable, False otherwise
        """
        d = np.linalg.norm(p)

        if d < 1e-5:
            return True
        y = np.dot(p,np.array([0,0,1]))
        if check_y and y < 0:
            return False
        x = d*np.sin(np.arccos(np.fmin(1,y/d)))
        centerx = 1/self.needle_lims[1,1] * np.cos(0.005)
        centery = -1/self.needle_lims[1,1] * np.sin(0.005)
        dist_to_center = np.linalg.norm(np.array([x-centerx, y - centery]))
        reach = dist_to_center >= 1/self.needle_lims[1,1] - 1e-5
        if (_DEBUG or print_)and not reach:
            print(f"dp: {np.round(p,4)} \treach: {reach} \td: {round(d, 4)} \tdist to center: {dist_to_center} \tlim: {1/self.needle_lims[1,1]} \tcenter: {centerx}, {centery}")

        return reach


    def get_lims(self, gw:npt.NDArray, print_:bool=False) -> npt.NDArray:
        """
        Calulates limits for needle_lims based on position and orientation in gw.

        Parameters:
            gw (4x4 ndarray): transformation matrix for the new point in the world frame
            print_ (bool): debugging print flag if True, default is False
            
        Returns:
            lims (3x2 ndarray): [[arc min, arc max], [curvature min, curvature max], [theta min, theta max]]
        """
        if self.variable_curvature:
            # get the closest point on the skull from the screw, and its distance
            screwmagdipole = np.array([[gw[0,2]],[gw[1,2]],[gw[2,2]]])
            normal_vec = np.cross(self.gw[0:3,2], screwmagdipole, axis=0)
            # print(f"normal: {normal_vec.reshape(-1,)} zold: {self.gw[0:3,2].reshape(-1,)} znew: {gw[0:3,2].reshape(-1,)}")
            if np.linalg.norm(normal_vec) < 1e-5:
                manipmagdipole = np.array([[self.gw[0,0]],[self.gw[1,0]],[self.gw[2,0]]])
                normal_vec = manipmagdipole
            normal_vec = normal_vec/np.linalg.norm(normal_vec)            
            skullpoint1, skullpoint2 = np.asarray(closestPoint(self.skull_tree, gw[0:3,3], normal_vec))

            # for the control magnet, just get an orthogonal vector
            
                
            other_vector = np.array([[gw[0,0]], [gw[1,0]], [gw[2,0]]])
            norm_m = screwmagdipole/np.linalg.norm(screwmagdipole)
            manipmagdipole = np.cross(normal_vec, screwmagdipole, axis=0)
            

            # has to be transposed because KD-tree expects 1x3 while Magnet Class expects 3x1
            skulltranspose = skullpoint1.reshape(3,1)
            screwtranspose = gw[0:3,3].reshape(3,1)

            
            manipMag1 = Magnet(skulltranspose, manipmagdipole, _MANIPMAG_STRENGTH)
            screwMag = Magnet(screwtranspose, screwmagdipole, _SCREWMAG_STRENGTH)

            f1, tau1 = screwMag.get_force_torque(manipMag1)

            maxCurvature1 = (self.r_curvature_line[0] * np.linalg.norm(tau1) + self.r_curvature_line[1])/1000
            manipMag2 = Magnet(skullpoint2.reshape(3,1), manipmagdipole, _MANIPMAG_STRENGTH)
            f2, tau2 = screwMag.get_force_torque(manipMag2)
            maxCurvature2 = (self.r_curvature_line[0] * np.linalg.norm(tau2) + self.r_curvature_line[1])/1000
            if maxCurvature1 > maxCurvature2:
                maxCurvature = maxCurvature1
                manipMag = manipMag1
                tau = tau1
                skullpoint = skullpoint1
                # print(f"not chosen lims.. skull point: {np.round(skullpoint2,4)}  |r|: {round(np.linalg.norm(skullpoint2 - gw[0:3, 3]),4)} lim: {round(1/maxCurvature2,10)} |tau|: {round(np.linalg.norm(tau2),10)}")
            else:
                maxCurvature = maxCurvature2
                manipMag = manipMag2
                tau = tau2
                skullpoint = skullpoint2
                # print(f"not chosen lims.. skull point: {np.round(skullpoint1,4)}  |r|: {round(np.linalg.norm(skullpoint1 - gw[0:3, 3]),4)} lim: {round(1/maxCurvature1,10)} |tau|: {round(np.linalg.norm(tau1),10)}")
            if _DEBUG or print_:
                print(f"skull point: {np.round(skullpoint,4)}  |r|: {round(np.linalg.norm(skullpoint - gw[0:3, 3]),4)} lim: {round(1/maxCurvature,10)} |tau|: {round(np.linalg.norm(tau),10)}")
                print(f"manip: {manipMag.m.reshape(-1,)/np.linalg.norm(manipMag.m)} needle: {screwMag.m.reshape(-1,)/np.linalg.norm(screwMag.m)} x: {gw[0:3,0].reshape(-1,)} y: {gw[0:3,1].reshape(-1,)} r: {(skullpoint - gw[0:3, 3])/np.linalg.norm(skullpoint - gw[0:3, 3])}")
            if maxCurvature > _MAXK:
                return np.array([[self.needle_lims[0,0], self.needle_lims[0,1]], [0, _MAXK], [self.needle_lims[2,0], self.needle_lims[2,1]]])
            else:
                # print(f"needle limits changed! curvature: {maxCurvature} old: {self.needle_lims[1,1]} p: {gw[0:3,3]} r curve line: {self.r_curvature_line}")
                return np.array([[self.needle_lims[0,0], self.needle_lims[0,1]], [0, maxCurvature], [self.needle_lims[2,0], self.needle_lims[2,1]]])
        else:  
            return self.needle_lims
        

    def get_new_lims(self, p:npt.NDArray, print_:bool=False):
        """
        Updates the curvature limits for the needle given the new target point.

        Parameters:
            p (3x1 ndarray): the new target point for the needle in the world frame
            print_ (bool): debugging print flag if True, default is False

        """
        if self.variable_curvature:
            sg = p - self.p
            sg_hat = sg/np.linalg.norm(sg)
            # get the closest point on the skull from the screw, and its distance
            

            # for the control magnet, just get an orthogonal vector
            screwmagdipole = np.array([[self.gw[0,2]],[self.gw[1,2]],[self.gw[2,2]]])
                
            # other_vector = np.array([[self.gw[0,0]], [self.gw[1,0]], [self.gw[2,0]]])
            # norm_m = screwmagdipole/np.linalg.norm(screwmagdipole)
           
            # manipmagdipole = np.cross(sg_hat, screwmagdipole, axis=0)
            
            # if print_:
            #     print(f"normal: {manipmagdipole.reshape(3,)} unit normal: {manipmagdipole.reshape(3,)/np.linalg.norm(manipmagdipole)} sghat: {sg_hat} ")

            temp_lim = self.needle_lims[1,1]
            self.needle_lims[1,1] = _MAXK
            q,phi = self.ik(p,print_=print_)
            if q is None:
                return

            theta = q[2]
            # xm = np.array([sin(theta), cos(theta), 0, 0])
            # ym = np.array([-cos(theta), sin(theta), 0, 0])
            # zm = np.array([0, 0, 1, 0])
            # dm = np.array([0, 0, 0, 1])

            # gm_old = np.transpose(np.vstack((xm, ym, zm, dm)))
            # if print_:
            #     print(gm)
            #     print(np.matmul(self.gw, gm))

            xm = np.array([-sin(theta), cos(theta), 0, 0])
            ym = np.array([-cos(theta)*cos(phi), -sin(theta)*cos(phi), sin(phi), 0])
            zm = np.array([cos(theta)*sin(phi), sin(theta)*sin(phi), cos(phi), 0])
            dm = np.array([0, 0, 0, 1])

            gm = np.transpose(np.vstack((xm, ym, zm, dm)))
            gw_new = np.matmul(self.gw, gm)
            if print_:
                print(gw_new) 

            normal_vec = np.cross(self.gw[0:3,2], gw_new[0:3,2], axis=0)
            # print(f"normal: {normal_vec.reshape(-1,)} zold: {self.gw[0:3,2].reshape(-1,)} znew: {gw_new[0:3,2].reshape(-1,)}")

            if np.linalg.norm(normal_vec) < 1e-5:
                manipmagdipole = np.array([[self.gw[0,0]],[self.gw[1,0]],[self.gw[2,0]]])
                normal_vec = manipmagdipole
            normal_vec = normal_vec/np.linalg.norm(normal_vec)
            skullpoint1, skullpoint2 = np.asarray(closestPoint(self.skull_tree, self.gw[0:3,3], normal_vec))

            manipmagdipole = np.cross(normal_vec, self.gw[0:3,2], axis=0)
            # has to be transposed because KD-tree expects 1x3 while Magnet Class expects 3x1
            skulltranspose = skullpoint1.reshape(3,1)
            screwtranspose = self.gw[0:3,3].reshape(3,1)

            
            manipMag1 = Magnet(skulltranspose, manipmagdipole, _MANIPMAG_STRENGTH)
            screwMag = Magnet(screwtranspose, screwmagdipole, _SCREWMAG_STRENGTH)

            f1, tau1 = screwMag.get_force_torque(manipMag1)

            maxCurvature1 = (self.r_curvature_line[0] * np.linalg.norm(tau1) + self.r_curvature_line[1])/1000
            manipMag2 = Magnet(skullpoint2.reshape(3,1), manipmagdipole, _MANIPMAG_STRENGTH)
            f2, tau2 = screwMag.get_force_torque(manipMag2)
            maxCurvature2 = (self.r_curvature_line[0] * np.linalg.norm(tau2) + self.r_curvature_line[1])/1000
            if maxCurvature1 > maxCurvature2:
                maxCurvature = maxCurvature1
                manipMag = manipMag1
                tau = tau1
                skullpoint = skullpoint1
                # print(f"not chosen lims.. skull point: {np.round(skullpoint2,4)}  |r|: {round(np.linalg.norm(skullpoint2 - self.gw[0:3, 3]),4)} lim: {round(1/maxCurvature2,10)} |tau|: {round(np.linalg.norm(tau2),10)}")
            else:
                maxCurvature = maxCurvature2
                manipMag = manipMag2
                tau = tau2
                skullpoint = skullpoint2
                # print(f"not chosen lims.. skull point: {np.round(skullpoint1,4)}  |r|: {round(np.linalg.norm(skullpoint1 - self.gw[0:3, 3]),4)} lim: {round(1/maxCurvature1,10)} |tau|: {round(np.linalg.norm(tau1),10)}")
            if _DEBUG or print_:
                print("new lim calcs")
                print(f"skull point: {np.round(skullpoint,4)}  |r|: {round(np.linalg.norm(skullpoint - self.gw[0:3, 3]),4)} lim: {round(1/maxCurvature,10)} |tau|: {round(np.linalg.norm(tau),10)} sample: {p} p: {self.p}")
                print(f"manip: {manipMag.m.reshape(-1,)/np.linalg.norm(manipMag.m)} needle: {screwMag.m.reshape(-1,)/np.linalg.norm(screwMag.m)} x: {self.gw[0:3,0].reshape(-1,)} y: {self.gw[0:3,1].reshape(-1,)} r: {(skullpoint - self.gw[0:3, 3])/np.linalg.norm(skullpoint - self.gw[0:3, 3])}")
            if not np.isnan(maxCurvature):
                if maxCurvature > _MAXK:
                    maxCurvature = _MAXK
                self.needle_lims[1,1] = maxCurvature



    def move_needle(self, p:npt.NDArray, q:tuple[float,float,float]=None, phi:float=0.0, print_:bool=False) -> 'SteerableNeedle':
        """
        "Moves" the current needle to the new desired position p if reachable.

        Parameters:
            p (array): [x, y, z] desired location of the needle in world frame
            q (tuple[float, float, float] | None): the control action to move the needle with, default is None and acction will be found
            phi (float): new phi amount between this needle and the new needle being created, default is 0.0 and will be found if q is None
            print_ (bool): debugging print flag if True, default is False
        
        Returns:
            new_needle (SteerableNeedle | None): a new instance with the qualities resulting from moving the needle to p
                        None if the point p is not reachable with needle constraints
        """
        if q is None:
            q, phi = self.ik(p, print_=print_)
        if q is not None:
            gw = self.fk(q)
            new_lims = self.get_lims(gw, print_=print_)
            new_needle = SteerableNeedle(new_lims, gw=gw, q=q, phi=self.phi+phi, l=self.l+q[0], phi_constraint=self.phi_constraint, skull_tree=self.skull_tree, r_curvature_line=self.r_curvature_line, variable_curvature=self.variable_curvature)
            return new_needle
        else:
            return None


    def draw_fk(self, q:tuple[float,float,float], color:str='b', show:bool=False):
        """
        Draw the needle with the provided configuration advancing using control action q.

        Parameters:
            q (tuple[float, float, float]): the control action to advance the needle
            color (str): the color to draw the robot point, default is 'b' (blue)
            show (bool): shows the plot with blocking if true, default is false
        """
        g = self.fk(q)
        pts = g[0:3,3]

        plt.plot(pts[0], pts[1], pts[3], color)
        if show:
            plt.show(block=True)


    def draw(self, p:npt.NDArray, color:str='b', show:bool=False):
        """
        Draw the robot with the provided configuration/location p.

        Parameters:
            p (3x1 ndarray): the point to draw the robot at
            color (str): the color to draw the robot point, default is 'b' (blue)
            show (bool): shows the plot with blocking if true, default is false
        """

        plt.plot(p[0], p[1], p[2], color)
        if show:
            plt.show(block=True)


    def change_p(self, p:npt.NDArray):
        """
        Changes the needle's p, updating the transformation matrices in the process.

        Parameters:
            p (3x1 ndarray): the new location for the needle in world coordinates
        """
        self.gw[0:3,3] = p
        self.change_gw(self.gw)


    def change_gw(self, gw:npt.NDArray):
        """
        Changes the needle's gw, updating the p and transformation matrices in the process.

        Parameters:
            gw (4x4 ndarray): the new tranformation matrix for the needle, containing the new location
        """
        self.gw = gw
        self.gw_inv = np.linalg.inv(self.gw)
        self.p = gw[0:3,3]



# LIKELY NEEDS TO BE CHANGED
def closestPoint(skulltree:KDTree, position:npt.NDArray, normal_vec:npt.NDArray) -> npt.NDArray:
    """
    Given a skull segmentation and a point in 3D, returns the closest point on the skull to that point and its distance.

    Parameters:
        skulltree (kdtree): kd tree made from a segmentation of the patients skull
        position (1x3 numpy array): 3D point in the skull to find distance to
    
    Returns:
        skullpoint (1x3 ndarray): point of skull closest to position, with an extra amount of padding added for safety
    """
    padding = 25.4 # CHANGE this value to reflect real world, also might not be needed here
    # may need to convert the frame of points IMPORTANT
    normal_vec = normal_vec.reshape(-1,)
    dist_, ind= skulltree.query(position.reshape(1, -1), k = 1, return_distance=True) 
    skull_point_pos = skulltree.data[ind[0,0]] 
    skull_point_neg = skulltree.data[ind[0,0]] 
    r_ = skull_point_pos - position
    r_hat_ = r_/np.linalg.norm(r_)
    diff = np.dot(r_hat_, normal_vec)
    diff_pos = diff
    diff_neg = diff
    dist_pos = dist_
    dist_neg = dist_

    for i in range(-100, 110, 10):
        position_ = position + i*normal_vec

        dist_, ind= skulltree.query(position_.reshape(1, -1), k = 1, return_distance=True)
        skullpoint_ = np.asarray(skulltree.data[ind[0,0]])
        r_ = skullpoint_ - position
        r_hat_ = r_/np.linalg.norm(r_)
        diff = np.dot(r_hat_, normal_vec)
        if diff < 0 and diff < diff_neg:
            dist_neg = np.linalg.norm(skullpoint_ - position)
            diff_neg = diff
            skull_point_neg = skullpoint_
        elif diff > 0 and diff > diff_pos:
            dist_pos = np.linalg.norm(skullpoint_ - position)
            diff_pos = diff
            skull_point_pos = skullpoint_
        # print(f"i: {i} pos: {position_.reshape(-1,)} skull: {skullpoint_} dist: {dist_[0][0]} diff: {diff} r: {r_} r_hat: {r_hat_}")
    # skullpoint = skulltree.data[ind[0,0]] 

    # if abs(diff_pos) - abs(diff_neg) < 0.01:
    #     if dist_neg < dist_pos:
    #         skullpoint = skull_point_neg
    #     else:
    #         skullpoint = skull_point_pos
    # elif abs(diff_pos) > abs(diff_neg):
    #     skullpoint = skull_point_pos
    # else:
    #     skullpoint = skull_point_neg
    # TODO: might need to change this to just return both options and check which leads to a better limit
    r = skull_point_pos - position
    r_mag = np.linalg.norm(r)
    r_hat = r/r_mag

    r_mag += padding
    # print(f"og point: [{round(skullpoint[0],6)} {round(skullpoint[1],6)} {round(skullpoint[2],6)}] dist: {round(dist,6)} diff:{np.round(diff,6)} position: {position} dist: {dist_}")
    skull_point_pos = position + r_mag*r_hat
    dist_, ind= skulltree.query(skull_point_pos.reshape(1, -1), k = 1, return_distance=True)
    while dist_[0][0] < padding:
        r_mag += 1
        skull_point_pos = position + r_mag*r_hat
        dist_, ind= skulltree.query(skull_point_pos.reshape(1, -1), k = 1, return_distance=True)

    r = skull_point_neg - position
    r_mag = np.linalg.norm(r)
    r_hat = r/r_mag

    r_mag += padding
    # print(f"og point: [{round(skullpoint[0],6)} {round(skullpoint[1],6)} {round(skullpoint[2],6)}] dist: {round(dist,6)} diff:{np.round(diff,6)} position: {position} dist: {dist_}")
    skull_point_neg = position + r_mag*r_hat
    dist_, ind= skulltree.query(skull_point_neg.reshape(1, -1), k = 1, return_distance=True)

    while dist_[0][0] < padding:
        r_mag += 1
        skull_point_neg = position + r_mag*r_hat
        dist_, ind= skulltree.query(skull_point_neg.reshape(1, -1), k = 1, return_distance=True)   

        
    return skull_point_pos, skull_point_neg