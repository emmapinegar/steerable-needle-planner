import numpy as np
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
    def __init__(self, needle_lims=None, p=None, gw=None, q=None, phi=0, l=0, phi_constraint=False, skull_tree=None, r_curvature_line=None, variable_curvature=False):
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
        self.variable_curvature = variable_curvature

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

    def ik(self, p, print_=False):
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
        print_str = ""
        if _DEBUG or print_:
            print_str = print_str + f"sample: {np.round(p.reshape(3,),15)} parent: {np.round(self.p.reshape(3,),15)} dp: {np.round(dp[0:3],15)}"
        if not self.reachable(dp[0:3], print_=print_):
            if _DEBUG or print_: 
                print(f"not reachable {print_str} k: {self.needle_lims[1,1]}")
            return None, None
        if np.linalg.norm(dp[0:3]) < 5e-10:
            # print(f"close enough sample: {np.round(p.reshape(3,),4)} parent: {np.round(self.p.reshape(3,),4)} dp: {np.round(dp[0:3],10)}")
            return (0, 0, 0), 0

        theta = atan2(py,px) 
        xy_sq = np.linalg.norm(dp[0:2])

        k = (2*xy_sq)/(np.sum(np.square(dp[0:3])))

        # if xy_sq < 1e-4:
        #     print(xy_sq)
        #     theta = np.pi/2 - theta
        #     k = 0.0

        # if k < 1e-3:
        #     k = 0.0

        # if xy_sq < 5e-3 and k < 1e-4 and (abs(theta) < np.pi/2-1e-2 or abs(theta) > np.pi/2 + 1e-2) :
            
        #     print(f"meets conditions sample: {np.round(p.reshape(3,),4)} parent: {np.round(self.p.reshape(3,),4)} dp: {np.round(dp[0:3],15)} xy_sq: {round(xy_sq,15)} k: {round(k,10)} theta: {round(theta,10)}")
        #     k = 0.0
            
            # if theta < 0.5:
            #     theta = -np.pi/2 #- theta
            # else:
            #     theta = np.pi/2

            # if abs(p[0] + 54.8557) < 1e-3 and abs(p[1] - 9.2278) < 1e-3 and abs(p[2] - 35.8112) < 1e-3 and xy_sq < 1e-3:
            #     # print(f"meets conditions sample: {np.round(p.reshape(3,),4)} parent: {np.round(self.p.reshape(3,),4)} dp: {np.round(dp[0:3],10)} xy_sq: {round(xy_sq,10)} k: {round(k,10)} theta: {round(theta,10)}")
            #     k = 0.0
            #     theta = -np.pi/2
            # if abs(p[0] + 53.81) < 1e-3 and abs(p[1] - 9.7224) < 1e-3 and abs(p[2] - 35.8112) < 1e-3 and xy_sq < 1e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 57.7125) < 1e-3 and abs(p[1] - 8.0703) < 1e-3 and abs(p[2] - 36.0446) < 1e-3 and xy_sq < 1e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 55.183) < 1e-3 and abs(p[1] - 10.3066) < 1e-3 and abs(p[2] - 39.6491) < 1e-3 and xy_sq < 1e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 54.3057) < 1e-3 and abs(p[1] - 7.4147) < 1e-3 and abs(p[2] - 37.161) < 1e-3 and xy_sq < 5e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 56.1111) < 1e-3 and abs(p[1] - 13.2681) < 1e-3 and abs(p[2] - 39.583) < 1e-3 and xy_sq < 5e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 55.5102) < 1e-3 and abs(p[1] - 11.3855) < 1e-3 and abs(p[2] - 43.4869) < 1e-3 and xy_sq < 5e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 56.7201) < 1e-3 and abs(p[1] - 13.8774) < 1e-3 and abs(p[2] - 39.1084) < 1e-3 and xy_sq < 5e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # elif abs(p[0] + 54.0318) < 1e-3 and abs(p[1] - 6.5118) < 1e-3 and abs(p[2] - 34.7374) < 1e-3 and xy_sq < 5e-3:
            #     k = 0.0
            #     theta = -np.pi/2
            # else:
                # if print_:
                
                # k = 0.0
                # theta = -np.pi/2


        if k == 0.0 or k < 1e-3:
            # print(f"going straight {p} theta: {theta}")
            l = pz
            k = 0.0
            phi = 0.0
            theta = -np.pi/2
            r = inf
        else:
            r = (np.sum(np.square(dp[0:3])))/(2*xy_sq)
            k = 1/r
            phi = atan2(pz, r - xy_sq)
            # if _DEBUG or print_:
            #     print(f"phi: {phi} \tpz: {pz} \tr: {r} \txy_sq: {xy_sq} r-xy: {r-xy_sq}")
            if phi < 0:
                phi = 2*np.pi + phi
            l = phi*r

        # arc length limit
        if self.l + l > self.needle_lims[0,1]:
            if _DEBUG or print_:
                print(f"violates insertion length! {self.l + l} {print_str}")
            return None, None

        # 90 degree constraint
        if self.phi_constraint and self.phi + phi > _MAXPHI:
            if _DEBUG or print_:
                print(f"phi rejected! {self.phi + phi} {print_str}" )
            return None, None
        if _DEBUG or print_:
            print(f"{print_str} l: {round(l,4)} \tk: {round(k,10)} \ttheta: {round(theta,10)} \tr: {round(r,4)} \tphi: {round(phi,4)}")   
        q = (l, k, theta)
        return q, phi
  
    def get_distance(self, p, print_=False):
        '''
        Get the distance to point p based on the needles pose
        '''
        dp = np.matmul(self.gw_inv, np.transpose(np.append(p,1)))
        px = dp[0]
        py = dp[1]
        pz = dp[2]
        if np.linalg.norm(dp[0:3]) < 5e-10:
            # print(f"close enough sample: {np.round(p.reshape(3,),4)} parent: {np.round(self.p.reshape(3,),4)} dp: {np.round(dp[0:3],10)} d: {np.linalg.norm(dp[0:3])}")
            return 0.0
        
        q, phi = self.ik(p)
        if q is None:
            distance = 10000
        else:
            distance = q[0]
            # if abs(q[1]) < 1e-2:
            #     # print(f"radius: {q[1]}")
            #     distance = q[0]
            # elif abs(1/q[1] - 14) < 1e-2:
            #     # print(f"radius: {q[1]} {1/q[1]}")
            #     distance = q[0]
            # else:
            #     # print(f"incorrect radius: {q[1]} {1/q[1]}")
            # # print(f"p: {p.reshape(-1,)} l: {q[0]} k: {q[1]} r: {1/q[1]}")
            #     distance = 10000
        # can_reach = self.reachable(dp[0:3], print_=print_)  

        # if can_reach:
        #     distance = np.linalg.norm(self.p - p)
        # else:
        #     distance = 10000
        return distance     

   
    def reachable(self, p, print_=False, check_y=False, oldcheck=False):
        '''
        Tests if a point p is reachable by the current pose of the needle given the curvature limits
        p (array): [x, y, z] desired location of the needle in the current needle frame
        Returns True if the point is reachable, False otherwise
        '''
        if oldcheck:
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
        else:
            d = np.linalg.norm(p)

            if d < 1e-5:
                return True
            y = np.dot(p,np.array([0,0,1]))
            if check_y and y < 0:
                return False
            x = d*np.sin(np.arccos(np.fmin(1,y/d)))
            dist_to_center = np.linalg.norm(np.array([x-1/self.needle_lims[1,1], y]))
            reach = dist_to_center >= 1/self.needle_lims[1,1] - 5e-2
            if (_DEBUG or print_)and not reach:
                print(f"dp: {np.round(p,4)} \treach: {reach} \td: {round(d, 4)} \tdist to center: {round(dist_to_center,4)} \tlim: {round(1/self.needle_lims[1,1],4)}") #\ty: {round(y,4)} \tx: {round(x,4)}

            # if not reach:
            #     print(f"p: {np.round(p,4)} \treach: {reach} \td: {round(d, 4)} \tdist to center: {round(dist_to_center,4)} \tlim: {round(1/self.needle_lims[1,1],4)}") #\ty: {round(y,4)} \tx: {round(x,4)}                
            return reach


    def get_lims(self, gw, print_=False):
        '''
        Calulates limits for needle_lims based on position p.
        p (array): [x, y, z] position of needle in BLANK frame
        Returns:
        lims (array): [[arc min, arc max], [curvature min, curvature max], [theta min, theta max]]
        '''
        if self.variable_curvature:
            # get the closest point on the skull from the screw, and its distance
            skullpoint = np.asarray(closestPoint(self.skull_tree, gw[0:3,3]))

            # for the control magnet, just get an orthogonal vector
            screwmagdipole = np.array([[gw[0,2]],[gw[1,2]],[gw[2,2]]])
                
            other_vector = np.array([[gw[0,0]], [gw[1,0]], [gw[2,0]]])
            norm_m = screwmagdipole/np.linalg.norm(screwmagdipole)
            manipmagdipole = other_vector #np.cross(norm_m, other_vector, axis=0)
            manipmagdipole = manipmagdipole/np.linalg.norm(manipmagdipole)

            # has to be transposed because KD-tree expects 1x3 while Magnet Class expects 3x1
            skulltranspose = skullpoint.reshape(3,1)
            screwtranspose = gw[0:3,3].reshape(3,1)

            
            manipMag = Magnet(skulltranspose, manipmagdipole, _MANIPMAG_STRENGTH)
            screwMag = Magnet(screwtranspose, screwmagdipole, _SCREWMAG_STRENGTH)

            f, tau = screwMag.get_force_torque(manipMag)
            # print(f"f: {f.reshape(-1,)} tau: {tau.reshape(-1)}")
            maxCurvature = (self.r_curvature_line[0] * np.linalg.norm(tau) + self.r_curvature_line[1])/1000
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
        

    def get_new_lims(self, p, print_=False):
        """
        
        """
        if self.variable_curvature:
            sg = p - self.p
            sg_hat = sg/np.linalg.norm(sg)
            # get the closest point on the skull from the screw, and its distance
            skullpoint = np.asarray(closestPoint(self.skull_tree, self.gw[0:3,3]))

            # for the control magnet, just get an orthogonal vector
            screwmagdipole = np.array([[self.gw[0,2]],[self.gw[1,2]],[self.gw[2,2]]])
                
            # other_vector = np.array([[self.gw[0,0]], [self.gw[1,0]], [self.gw[2,0]]])
            # norm_m = screwmagdipole/np.linalg.norm(screwmagdipole)
            manipmagdipole = np.cross(sg_hat, screwmagdipole, axis=0)
            manipmagdipole = manipmagdipole/np.linalg.norm(manipmagdipole)

            # test =  self.gw[0:3,3].reshape(3,) - 10*manipmagdipole.reshape(3,)
            # test = test.reshape(3,1)
            # print(test)
            # get the closest point on the skull from the screw, and its distance
            # skullpoint = np.asarray(closestPoint(self.skull_tree, test))

            # has to be transposed because KD-tree expects 1x3 while Magnet Class expects 3x1
            skulltranspose = skullpoint.reshape(3,1)
            screwtranspose = self.gw[0:3,3].reshape(3,1)

            
            manipMag = Magnet(skulltranspose, manipmagdipole, _MANIPMAG_STRENGTH)
            screwMag = Magnet(screwtranspose, screwmagdipole, _SCREWMAG_STRENGTH)

            f, tau = screwMag.get_force_torque(manipMag)
            # print(f"f: {f.reshape(-1,)} tau: {tau.reshape(-1)}")
            maxCurvature = (self.r_curvature_line[0] * np.linalg.norm(tau) + self.r_curvature_line[1])/1000
            if _DEBUG or print_:
                print("new lim calcs")
                print(f"skull point: {np.round(skullpoint,4)}  |r|: {round(np.linalg.norm(skullpoint - self.gw[0:3, 3]),4)} lim: {round(1/maxCurvature,10)} |tau|: {round(np.linalg.norm(tau),10)} sample: {p} p: {self.p}")
                print(f"manip: {manipMag.m.reshape(-1,)/np.linalg.norm(manipMag.m)} needle: {screwMag.m.reshape(-1,)/np.linalg.norm(screwMag.m)} x: {self.gw[0:3,0].reshape(-1,)} y: {self.gw[0:3,1].reshape(-1,)} r: {(skullpoint - self.gw[0:3, 3])/np.linalg.norm(skullpoint - self.gw[0:3, 3])}")
            if not np.isnan(maxCurvature):
                if maxCurvature > _MAXK:
                    maxCurvature = _MAXK
                self.needle_lims[1,1] = maxCurvature



    def move_needle(self, p, q=None, phi=0.0, print_=False):
        '''
        "Moves" the current needle to the new desired position p if reachable.
        p (array): [x, y, z] desired location of the needle in world frame
        Returns:
        new_needle (SteerableNeedle): a new instance with the qualities resulting from moving the needle to p
                        None if the point p is not reachable with needle constraints
        '''
        if q is None:
            q, phi = self.ik(p, print_=print_)
        if q is not None:
            gw = self.fk(q)
            new_lims = self.get_lims(gw, print_=print_)
            # if _DEBUG:
            #     print(f"p: {gw[0:3,3]} q: {q} new lims: {new_lims.reshape(-1,)}")
            new_needle = SteerableNeedle(new_lims, gw=gw, q=q, phi=self.phi+phi, l=self.l+q[0], phi_constraint=self.phi_constraint, skull_tree=self.skull_tree, r_curvature_line=self.r_curvature_line, variable_curvature=self.variable_curvature)
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



# LIKELY NEEDS TO BE CHANGED
def closestPoint(skulltree:KDTree, position):
    """
    Given a skull segmentation and a point in 3D, returns the closest point on the skull to that point and its distance.
    Parameters:
    skulltree (kdtree): kd tree made from a segmentation of the patients skull
    position (1x3 numpy array): 3D point in the skull to find distance to
    
    Returns:
    distance (float64): distance from the given position to the closest point on the skull
    skullPoint (1x3 numpy array):

    """
    padding = 20 # CHANGE this value to reflect real world, also might not be needed here
    # may need to convert the frame of points IMPORTANT
    
    dist_, ind= skulltree.query(position.reshape(1, -1), k = 1, return_distance=True) 
    # p1 = skulltree.data[ind[0,0]]
    # p2 = skulltree.data[ind[0,1]]
    # p3 = skulltree.data[ind[0,2]]
    # p4 = skulltree.data[ind[0,3]]
    # p5 = skulltree.data[ind[0,4]]
    # p6 = skulltree.data[ind[0,5]]
    # p7 = skulltree.data[ind[0,6]]
    # p8 = skulltree.data[ind[0,7]]
    # p9 = skulltree.data[ind[0,8]]
    # p10 = skulltree.data[ind[0,9]]
    # print(f"{p1[0]} {p1[1]} {p1[2]}")
    # print(f"{p2[0]} {p2[1]} {p2[2]}")
    # print(f"{p3[0]} {p3[1]} {p3[2]}")
    # print(f"{p4[0]} {p4[1]} {p4[2]}")
    # print(f"{p5[0]} {p5[1]} {p5[2]}")
    # print(f"{p6[0]} {p6[1]} {p6[2]}")
    # print(f"{p7[0]} {p7[1]} {p7[2]}")
    # print(f"{p8[0]} {p8[1]} {p8[2]}")
    # print(f"{p9[0]} {p9[1]} {p9[2]}")
    # print(f"{p10[0]} {p10[1]} {p10[2]}")


    skullpoint = skulltree.data[ind[0,0]] 

    diff = skullpoint - position
    dist = np.linalg.norm(diff)

    # print(f"og point: [{round(skullpoint[0],6)} {round(skullpoint[1],6)} {round(skullpoint[2],6)}] dist: {round(dist,6)} diff:{np.round(diff,6)} position: {position} dist: {dist_}")
    skullpoint = (padding + dist) * diff / dist + position
        # print(f"pos: {position} skull: {skullpoint} dist: {dist}")
        
    return skullpoint