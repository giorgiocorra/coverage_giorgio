import coverage_giorgio as cov
import numpy as np
import math
PI = math.pi
s = math.sin
c = math.cos

def landmarks():
	q_0 = [1.,0.,0.]
	q_1 = [2.,0.,0.]
	q_2 = [3.,0.,0.]
	psi_0 = PI/2.
	theta_0 = PI/6.
	psi_1 = PI/3.
	theta_1 = -PI/4.
	psi_2 = PI/3.
	theta_2 = 0.
	u_0 = unit_vec(psi_0, theta_0)
	u_1 = unit_vec(psi_1, theta_1)
	u_2 = unit_vec(psi_2, theta_2)
	lm_0 = cov.Landmark(q_0,u_0)
	lm_1 = cov.Landmark(q_1,u_1)
	lm_2 = cov.Landmark(q_2,u_2)
	return cov.Landmark_list([lm_0,lm_1,lm_2])

def position():
	return [4.,1.,1.]

def orientation():
	return [0.,-1.,0.]







""" FUNCTIONS ALREADY PRESENT IN utility_functions"""

def rot_y(tt):
    """This function returns the rotation matrix corresponding to a rotation
        of tt radians about the y-axis.
        """
    return np.array(
        [[c(tt), 0.0, s(tt)], [0.0, 1, 0.0], [-s(tt), 0.0, c(tt)]])


def rot_z(tt):
    """This function returns the rotation matrix corresponding to a rotation
        of tt radians about the z-axis.
        """
    return np.array(
        [[c(tt), -s(tt), 0.0], [s(tt), c(tt), 0.0], [0.0, 0.0, 1]])

def unit_vec(psi, theta):
    """This function returns the unit vector corresponding to the euler angles
    psi (about the z-axis) and theta (about the y'-axis).
    """
    e1 = np.array([1.0, 0.0, 0.0])
    aux = rot_z(psi).dot(e1)
    aux = rot_y(theta).dot(aux)
    return aux