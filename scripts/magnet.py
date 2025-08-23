
import numpy as np
import numpy.typing as npt

_DEBUG = False


# Define relevent values for magnet calulations
_MU_O = 4.0*np.pi*1e-7 # permeability of free space, should not be changed



class Magnet():

    def __init__(self, position:npt.NDArray, m:npt.NDArray, m_mag:float):
        """
        Class to define magnet objects.

        Parameters:
            position (3x1 ndarray): x,y,z position vector from global origin to center of magnet in millimeters
            m (3x1 ndarray): the dipole vector of the magnet, this will be normalized before making it of magnitide m_mag
            m_mag (float): the strength/magnitude of the magnet's dipole
        """
        self.position = np.array(position/1000) #conversion to mm
        m = m.reshape((3,1))
        self.m = np.array(m)
        norm = np.linalg.norm(self.m)
        self.m = np.divide(self.m, norm)
        self.m = np.multiply(m_mag, self.m)
        self.mag = m_mag
        self.skew_m = vector_to_skew(self.m)
        

    def get_Bb(self, other_magnet:'Magnet') -> tuple[npt.NDArray, npt.NDArray]:
        """
        Gets the field derivative and magnetic field at the location of this magnet.

        Parameters:
            other_magnet (Magnet): the other magnet

        Returns:
            [B,b] ([3x3 ndarray, 3x1 ndarray]): [field derivative matrix, magnetic field vector]
        """
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


    def get_r(self, other_magnet:'Magnet') -> tuple[npt.NDArray, float, npt.NDArray]:
        """
        Gets several vectors that look at the position difference between the two magnets.
        
        Parameters:
            other_magnet (Magnet): the other magnet

        Returns:
            [r,r_mag,r_hat] ([ndarray, float, ndarray]): [self.position - other_magnet.position, ||r||, r/||r||]
        """
        r = self.position - other_magnet.position
        r_mag = np.linalg.norm(r) #added padding
        r_hat = np.divide(r,r_mag)
        r_mag = r_mag #+ 0.02
        return r, r_mag, r_hat


    def get_force_torque(self, other_magnet:'Magnet') -> tuple[npt.NDArray, npt.NDArray]:
        """        
        Gets the force and torque vectors this magnet experiences because of other magnet.

        Parameters:
            other_magnet (Magnet): the other magnet in the interaction

        Returns:
            [f,tau] (ndarray, ndarray): force vector this magnet experiences , vector the torque is about and the magnitude
        """

        B, b = self.get_Bb(other_magnet)
        f = np.matmul(np.transpose(B), self.m)
        tau = np.matmul(self.skew_m, b)
        return f, tau
   

def vector_to_skew(vector:npt.NDArray) -> npt.NDArray:
    """
    Given a vector returns the skew symmetric matrix for that vector.

    Parameters:
        vector (3x1 ndarray): vector being turned into skew

    Returns: 
        skew (3x3 ndarray): skew symmetric matrix of vector
    """
    skew = np.array([[0, -vector[2,0], vector[1,0]], [vector[2,0], 0, -vector[0,0]], [-vector[1,0], vector[0,0], 0]])
    return skew