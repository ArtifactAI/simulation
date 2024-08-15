from scipy.spatial.transform import Rotation as R
import numpy as np
import control as ct

def airdata(V_body, V_wind_NED, quat_body_to_NED):
    """
    Compute vehicle velocity relative to the atmosphere.
    
    Inputs:
    V_body: 3x1 np.array
        Body frame velocity vector
    V_wind_NED: 3x1 np.array
        Velocity of moving atmosphere, expressed in the NED frame
    quat_body_to_NED: 4x1 np.array
        Quaternion (real part first) that expresses a body-frame vector in the NED frame. (R_e2b). Equivalently, it is the 3-2-1 (psi, theta, phi) rotation needed to align the NED frame with the body frame.

    Returns:
    tuple
        airspeed, alpha, beta, Va_body (body frame airspeed vector)
    """
    quat_body_to_NED = np.roll(quat_body_to_NED, -1) # Rotation package expects quat in (x, y, z, real) format
    rot_body_to_NED = R.from_quat(quat_body_to_NED)

    rot_NED_to_body = R.inv(rot_body_to_NED)

    V_wind_body = rot_NED_to_body.apply(V_wind_NED)

    # Compute airspeed vector
    Va_body = V_body - V_wind_body

    # Compute airspeed magnitude
    airspeed = np.linalg.norm(Va_body)
    
    # Compute alpha (angle of attack)
    alpha = np.arctan2(Va_body[2], Va_body[0])
    
    # Compute beta (sideslip angle)
    if airspeed > 0:
        # arbitrary value to avoid division by zero
        beta = np.arcsin(Va_body[1] / airspeed)
    else:
        beta = 0

    # from_euler rotates a vector, so expressing a vector in another frame means we use the negative
    # TODO: check correctness of this:
    # R2_alpha = R.from_euler('y', -alpha, degrees=False)
    # R3_neg_beta = R.from_euler('z', -beta, degrees=False)
    # R_wind2body = R2_alpha * R3_neg_beta
    
    return airspeed, alpha, beta, Va_body


def calc_airdata_outputs(t, x, u, params):
    V_body = u[:3]
    V_wind_NED = np.zeros(3) # TODO: add wind
    quat_body_to_NED = u[3:]

    if np.linalg.norm(quat_body_to_NED) > 0:
        # BUG: due to the way outputs are propagated through the system in python-control 
        # https://github.com/python-control/python-control/issues/1009
        quat_body_to_NED = quat_body_to_NED / np.linalg.norm(quat_body_to_NED)
        airspeed, alpha, beta, Va_body = airdata(V_body, V_wind_NED, quat_body_to_NED)
    else:
        airspeed, alpha, beta = 0, 0, 0
    
    return airspeed, alpha, beta

airdata_inputs = ['u', 'v', 'w', 'qw', 'qx', 'qy', 'qz']
airdata_outputs = ['airspeed', 'alpha', 'beta']
airdata_calcs = ct.nlsys(updfcn=None, outfcn=calc_airdata_outputs, inputs=airdata_inputs, outputs=airdata_outputs, name='airdata_calcs')
