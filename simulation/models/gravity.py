import control as ct
import numpy as np
from scipy.spatial.transform import Rotation as R

def calc_gravity_outputs(t, x, u, params):
    # return the gravity vector in the body frame

    g = 9.81

    if np.linalg.norm(u) > 0:
        quat_body_to_NED = np.roll(u, -1) # quaternion = x, y, z, w (real part last). The Rotation package assumes real part last, but the simulation puts it first.
        quat_body_to_NED = R.from_quat(quat_body_to_NED) # from vehicle state
        quat_NED_to_body = quat_body_to_NED.inv()
        gravity_body = quat_NED_to_body.apply([0,0,g])
    else:
        # BUG: due to the way outputs are propagated through the system in python-control 
        # https://github.com/python-control/python-control/issues/1009
        gravity_body = np.array([0,0,0])
    
    return params['mass']*gravity_body

gravity_inputs = ['qw', 'qx', 'qy', 'qz']
gravity_outputs = ['Fx_grav', 'Fy_grav', 'Fz_grav']
gravity = ct.nlsys(updfcn=None, outfcn=calc_gravity_outputs, inputs=gravity_inputs, outputs=gravity_outputs, name='gravity')