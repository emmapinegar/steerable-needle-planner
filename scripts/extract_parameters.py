
import os
import numpy as np
import matplotlib.pyplot as plt
from math import hypot, cos, sin, atan, atan2, pi, isnan, inf
import time
import json
from scipy.spatial.transform import Rotation as R

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
_OCCUPIED = 1




# Define relevent values for magnet calulations
_MU_O = 4.0*np.pi*1e-7 # permeability of free space, should not be changed
_MANIPMAG_STRENGTH = 66.03
_SCREWMAG_STRENGTH = 0.0018
# MOVE THESE LATER TO BE CHANGABLE (or just somewhere else)



class Magnet():

    def __init__(self, position, m, m_mag):
        '''
        Class to define magnet objects.
        Parameters:
        position (3x1 array): x,y,z position vector from global origin to center of magnet
        m (3x1 array): the dipole vector of the magnet, this will be normalized before making it of magnitide m_mag
        m_mag (double): the strength/magnitude of the magnet's dipole
        '''
        self.position = np.array(position/1000) #conversion to mm
        self.m = np.array(m)
        norm = np.linalg.norm(self.m)
        self.m = np.divide(self.m, norm)
        self.m = np.multiply(m_mag, self.m)
        self.mag = m_mag
        self.skew_m = vector_to_skew(self.m)
        
    def get_Bb(self, other_magnet):
        '''
        Gets the field derivative and magnetic field at the location of this magnet
        Parameters:
        other_magnet (Magnet): the other magnet

        Returns:
        B (3x3 numpy array): the field derivative matrix
        b (3x1 numpy array): the magnetic field 
        '''
        r_ij, r_mag, r_hat = self.get_r(other_magnet)

        coeff = _MU_O/(4.0*np.pi*r_mag**3)
        mi_rt = np.outer(other_magnet.m, r_hat)
        r_mit = np.outer(r_hat, other_magnet.m)
        rt_mi = np.inner(r_hat, other_magnet.m)
        r_outer = np.outer(r_hat, r_hat)
        matrix = np.eye(3) - 5.0*r_outer
        B = coeff*(3.0/r_mag)* (mi_rt + r_mit + rt_mi*matrix)
        b = coeff*(3.0*r_outer - np.eye(3))
        b = np.matmul(b, other_magnet.m)
        return B, b

    def get_r(self, other_magnet):
        '''
        Gets several vectors that look at the position difference between the two magnets
        Parameters:
        other_magnet (Magnet): the other magnet

        Returns:
        r (3x1 numpy array): self.position - other_magnet.position
        r_mag (double): ||r||
        r_hat (3x1 numpy array): r/||r||
        '''
        r = self.position - other_magnet.position
        r_mag = np.linalg.norm(r) + 20 #added padding
        r_hat = np.divide(r,r_mag)
        return r, r_mag, r_hat

    def get_force_torque(self, other_magnet):
        '''
        Gets the force and torque vectors this magnet experiences because of other magnet
        Parameters:
        other_magnet (Magnet): the other magnet in the interaction

        Returns:
        f (3x1 numpy array): force vector this magnet experiences
        tau (3x1 numpy array): vector the torque is about and the magnitude
        '''
        B, b = self.get_Bb(other_magnet)
        f = np.matmul(np.transpose(B), self.m)
        tau = np.matmul(self.skew_m, b)
        return f, tau





class SteerableNeedle:
    def __init__(self, needle_lims=None, p=None, gw=None, q=None, phi=0, l=0, phi_constraint=False, skull_tree=None, r_curvature_line=None):
        '''
        Steerable needle object that can be used to calculate forward kinematics, inverse kinematics, test reachability of an action, etc. 
        needle_lims (array): [[arc min, arc max], [curvature min, curvature max], [theta min, theta max]]
        p (array): [x, y, z] location of the needle in world frame, overriden by location specified in gw
        gw (array): transformation matrix of the needle from needle to world frame
        q (array): [l (arc), k (curvature), theta (angle)] the control action used to get to the current needle location/pose
        skull_tree (kdtree): kdtree used to find the closest point in the skull
        r_curvature_line (array): array holding values for defining the curvature best fit line
        '''
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
        self.skull_tree = skull_tree
        self.r_curvature_line = r_curvature_line

    def fk(self, q):
        '''
        Compute forward kinematics for the robot using action q
        q (array): [l (arc), k (curvature), theta (angle)] the action to apply to move the current needle
        Returns:
        g_new (array): a transformation matrix g_new from the resulting needle pose to world frame
        
        '''
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

    def ik(self, p):
        '''
        Compute inverse kinematics for the robot with position p
        p (array): [x, y, z] desired location of the needle in world frame
        Returns:
        q (array): [l (arc), k (curvature), theta (angle)] the action used to move the current needle to the specified p
                        None if the desired p is not reachable
        '''
        dp = np.matmul(self.gw_inv, np.transpose(np.append(p,1)))
        px = dp[0]
        py = dp[1]
        pz = dp[2]
        if _DEBUG:
            print(f"p: {p} dp: {dp[0:3]}")
        if not self.reachable(dp[0:3]):
            return None, None

        theta = atan2(py,px) 
        xy_sq = np.linalg.norm(dp[0:2])

        k = (2*xy_sq)/(np.sum(np.square(dp[0:3])))
        if k == 0.0:
            l = pz
            k = 0.0
            phi = 0.0           
            r = inf
        else:
            r = (np.sum(np.square(dp[0:3])))/(2*xy_sq)
            k = 1/r
            phi = atan2(pz, r - xy_sq)
            l = phi*r

        # arc length limit
        if self.l + l > self.needle_lims[0,1]:
            if _DEBUG:
                print(f"violates insertion length! {self.l + l}")
            return None, None

        # 90 degree constraint
        if self.phi_constraint and self.phi + phi > np.pi/2:
            if _DEBUG:
                print(f"phi rejected! {self.phi + phi}" )
            return None, None
        if _DEBUG:
            print(f"l: {round(l,4)} \tk: {round(k,4)} \ttheta: {round(theta,4)} \tr: {round(r,4)}")   
        q = (l, k, theta)
        return q, phi
  
    def get_distance(self, p):
        '''
        Get the distance to point p based on the needles pose
        '''
        dp = np.matmul(self.gw_inv, np.transpose(np.append(p,1)))
        px = dp[0]
        py = dp[1]
        pz = dp[2]
        if np.linalg.norm(dp[0:3]) < 1e-10:
            return 0.0
        can_reach = self.reachable(dp[0:3])  

        if can_reach:
            distance = np.linalg.norm(self.p - p)
        else:
            distance = 10000
        return distance     

   
    def reachable(self, p, print_=False):
        '''
        Tests if a point p is reachable by the current pose of the needle given the curvature limits
        p (array): [x, y, z] desired location of the needle in the current needle frame
        Returns True if the point is reachable, False otherwise
        '''
        xy_sq = p[0]**2 + p[1]**2
        
        if xy_sq > 0:
            r = (2/self.needle_lims[1,1]) * np.sqrt(xy_sq) - (xy_sq)
            if r < 0:
                if _DEBUG or print_:
                    print(f"dp: {p} r: {r} ")
                return False
            circle = np.sqrt(r)
            in_circle = p[2] >= circle
            if _DEBUG or print_:
                print(f"dp: {p} r: {r} circle: {circle} incircle: {in_circle}")
            return in_circle

        return True
    
    
    def get_lims(self, p):
        '''
        Calulates limits for needle_lims based on position p.
        p (array): [x, y, z] position of needle in BLANK frame
        Returns:
        lims (array): [[arc min, arc max], [curvature min, curvature max], [theta min, theta max]]
        '''
        
        # QUESTIONS
        # is the curvature in limits radius or kappa?
        
        # NOTES
        # need to get the graph readable as well
        # The screw dipole is the X axis, not y.
        
        # get the closest point on the skull from the screw, and its distance
        skullpoint = np.asarray(closestPoint(self.skull_tree, p)) # what frame is p in?
        
        # CHANGE these values based on real ones
        # for the control magnet, just get an orthogonal vector
        screwmagdipole = np.array([[self.gw[0,0]],[self.gw[1,0]],[self.gw[2,0]]])
            
        other_vector = np.array([[0], [0], [1]])
        norm_m = screwmagdipole/np.linalg.norm(screwmagdipole)
        manipmagdipole = np.cross(norm_m, other_vector, axis=0)
        manipmagdipole = manipmagdipole/np.linalg.norm(manipmagdipole)

        # has to be transposed because KD-tree expects 1x3 while Magnet Class expects 3x1
        skulltranspose = skullpoint.reshape(3,1)
        # uncomment the line underneath to break the code for testing
        #screwtranspose = p.reshape(1,3)
        screwtranspose = p.reshape(3,1)

        
        manipMag = Magnet(skulltranspose, manipmagdipole, _MANIPMAG_STRENGTH)
        screwMag = Magnet(screwtranspose, screwmagdipole, _SCREWMAG_STRENGTH)

        f, tau = screwMag.get_force_torque(manipMag)
        
        maxCurvature = (self.r_curvature_line[0] * np.linalg.norm(tau) + self.r_curvature_line[1])/1000
        #print(maxCurvature)
        #print(self.r_curvature_line[0])
        #print(np.linalg.norm(tau))
        #print(self.r_curvature_line[0] * np.linalg.norm(tau))
        #print(self.r_curvature_line[1])
        
        #return self.needle_lims
        
        if maxCurvature > 0.05:
            return np.array([[self.needle_lims[0,0], self.needle_lims[0,1]], [0, 0.05], [self.needle_lims[2,0], self.needle_lims[2,1]]])
        else:
            # print(f"needle limits changed! curvature: {maxCurvature} old: {self.needle_lims[1,1]}")
            return np.array([[self.needle_lims[0,0], self.needle_lims[0,1]], [0, maxCurvature], [self.needle_lims[2,0], self.needle_lims[2,1]]])

    def move_needle(self, p):
        '''
        "Moves" the current needle to the new desired position p if reachable.
        p (array): [x, y, z] desired location of the needle in world frame
        Returns:
        new_needle (SteerableNeedle): a new instance with the qualities resulting from moving the needle to p
                        None if the point p is not reachable with needle constraints
        '''
        q, phi = self.ik(p)
        if q is not None:
            gw = self.fk(q)
            new_lims = self.get_lims(self.p)
            new_needle = SteerableNeedle(new_lims, gw=gw, q=q, phi=self.phi+phi, l=self.l+q[0], phi_constraint=self.phi_constraint, skull_tree=self.skull_tree, r_curvature_line=self.r_curvature_line)
            return new_needle
        else:
            return None

    def draw_fk(self, q, color='b', show=False, base_color='g'):
        '''
        Draw the robot with the provided configuration advancing using control action q
        '''
        g = self.fk(q)
        pts = g[0:3,3]

        style = color+'o'
        plt.plot(pts[0], pts[1], pts[3], color)
        if show:
            plt.show(block=True)

    def draw(self, p, color='b', show=False, base_color='g'):
        '''
        Draw the robot with the provided configuration/location p
        '''
        style = color+'o'
        plt.plot(p[0], p[1], p[2], color)
        if show:
            plt.show(block=True)

    def change_p(self, p):
        '''
        Changes the p, updating the transformation matrices in the process
        '''
        self.gw[0:3,3] = p
        self.change_gw(self.gw)

    def change_gw(self, gw):
        '''
        Changes the gw, updating the p and transformation matrices in the process
        '''
        self.gw = gw
        self.gw_inv = np.linalg.inv(self.gw)
        self.p = gw[0:3,3]




def parse_file(filename):
    """
    
    """
    file = open(filename, 'r')
    lines = file.readlines()
    for line in lines:
        line_info = line.strip().split()
        if line_info[0].startswith('#'):
            continue
        data = parse_line(line_info)




def parse_line(line_data):
    """
    
    """
    point = np.array([line_data[0], line_data[1], line_data[2]])
    quat = [line_data[3], line_data[4], line_data[5], line_data[6]]
    rot = R.from_quat(quat, scalar_first=True)