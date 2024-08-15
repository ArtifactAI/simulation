from scipy.spatial.transform import Rotation as R
import numpy as np
import control as ct

def euler_angles(quaternion):
    """
    Compute aircraft Euler angles (yaw, pitch, roll) from quaternion
    Quaternion input should have (w, x, y, z) format (real part first)
    
    The quaternion rotates the NED frame to align with the body frame.
    Equivalently, the quaternion expresses a body-frame vector in the NED frame. 

    The resulting 'zyx' Euler rotation gives the sequence of rotations (yaw, pitch, roll) to align the NED frame to the body frame.
    """
    quaternion = np.roll(quaternion, -1) # quaternion = x, y, z, w (real part last). The Rotation package assumes real part last, but the simulation puts it first.
    quaternion = quaternion / np.linalg.norm(quaternion)
    r = R.from_quat(quaternion)
    euler_angles = r.as_euler('zyx', degrees=False)
    return euler_angles


def calc_euler_angles_outputs(t, x, u, params):
    """
    Compute aircraft Euler angles (yaw, pitch, roll) from quaternion
    Quaternion convention: x, y, z, w (real part last)
    The quaternion expresses a body-frame vector in the NED frame. Equivalently, the quaternion and resulting 'zyx' Euler rotation gives the sequence of rotations (yaw, pitch, roll) to align the NED frame to the body frame.
    """

    if np.all(u == 0):
        # BUG: Hacky fix for the case where u = 0 on every other timestep 
        # GitHub Issue: https://github.com/python-control/python-control/issues/1009
        return np.array([0.0, 0.0, 0.0])
    
    quaternion = np.roll(u, -1) # quaternion = x, y, z, w (real part last). The Rotation package assumes real part last, but the simulation puts it first.
    quaternion = quaternion / np.linalg.norm(quaternion)
    r = R.from_quat(quaternion)
    euler_angles = r.as_euler('zyx', degrees=False)
    return euler_angles

euler_angles_inputs = ['qw', 'qx', 'qy', 'qz']
euler_angles_outputs = ['psi', 'theta', 'phi']
euler_angles_calc = ct.nlsys(updfcn=None,outfcn=calc_euler_angles_outputs, inputs=euler_angles_inputs, outputs=euler_angles_outputs, name='euler_angles_calc')

