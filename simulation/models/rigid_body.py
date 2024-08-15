import control as ct
import numpy as np
from scipy.spatial.transform import Rotation as R

from .forces_moments import forces_moments

def dynamics_translation(t, x, u, params):
    # x is the velocity state
    # u[1:3] is the force vector
    # u[4:6] is the rotational rate

    if len(u) < 4:
        omega = np.zeros(3)
    else:
        omega = u[3:]  

    force = u[:3]   

    return force / params['mass'] - np.cross(omega, x)

def kinematics_translation(t, x, u, params):
    # x is the position state 
    # u[1:3] is the velocity vector

    V_body = u[:3]
    quat_body_to_NED = u[3:]
    quat_body_to_NED = np.roll(quat_body_to_NED, -1) # Rotation package expects quat in (x, y, z, real) format
    R_body_to_NED = R.from_quat(quat_body_to_NED)

    return R_body_to_NED.apply(V_body)

def dynamics_rotation(t, x, u, params):
    # x is the angular rates
    # u[1:3] are the moments

    J = params['inertia']
    return np.linalg.inv(J) @ u - np.cross(x, J @ x)

def kinematics_rotation(t, x, u, params):
    # x is the quaternion
    # u[1:3] is the angular velocity vector
    Q = np.array([[0, -u[0], -u[1], -u[2]],
                    [u[0], 0, u[2], -u[1]],
                    [u[1], -u[2], 0, u[0]],
                    [u[2], u[1], -u[0], 0]])
    return .5*Q @ x

translation_dynamics = ct.nlsys(dynamics_translation, outfcn=None, states=3, inputs=['Fx', 'Fy', 'Fz', 'p', 'q', 'r'], outputs=['u', 'v', 'w'], name='translation_dynamics')
translation_kinematics = ct.nlsys(kinematics_translation, outfcn=None, states=3, inputs=['u', 'v', 'w', 'qw', 'qx', 'qy', 'qz'], outputs=['x', 'y', 'z'], name='translation_kinematics')
rotation_dynamics = ct.nlsys(dynamics_rotation, outfcn=None, states=3, inputs=['Mx', 'My', 'Mz'], outputs=['p', 'q', 'r'], name='rotation_dynamics')
rotation_kinematics = ct.nlsys(kinematics_rotation, outfcn=None, states=4, inputs=['p', 'q', 'r'], outputs=['qw', 'qx', 'qy', 'qz'], name='rotation_kinematics')

rigid_body_inputs = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
rigid_body_outputs = ['x', 'y', 'z', 'u', 'v', 'w', 'qw', 'qx', 'qy', 'qz', 'p', 'q', 'r']

rigid_body = ct.interconnect(
    (translation_kinematics, translation_dynamics, rotation_kinematics, rotation_dynamics), # states are placed in this order
    name="rigid_body",
    inplist=rigid_body_inputs, inputs=rigid_body_inputs,
    outlist=rigid_body_outputs, outputs=rigid_body_outputs, # actual outer-system outputs
)

# test rigid_body_dynamics
if __name__ == "__main__":
    initial_state = {
        'x': 0.0,  # Initial x position
        'y': 0.0,  # Initial y position
        'z': 0.0,  # Initial z position
        'u': 0.0,  # Initial body-frame x-axis velocity
        'v': 0.0,  # Initial body-frame y-axis velocity
        'w': 0.0,  # Initial body-frame z-axis velocity
        'qw': 1.0, # Initial quaternion component (real part)
        'qx': 0.0, # Initial quaternion component (i)
        'qy': 0.0, # Initial quaternion component (j)
        'qz': 0.0, # Initial quaternion component (k)
        'p': 0.0,  # Initial roll rate
        'q': 0.0,  # Initial pitch rate
        'r': 0.0   # Initial yaw rate
    }

    x0 = np.array([initial_state[key] for key in [
        'x', 'y', 'z', 'u', 'v', 'w', 'qw', 'qx', 'qy', 'qz', 'p', 'q', 'r'
    ]])

    # quaternion convention is to give the rotation of the NED frame to the body frame. In other words, the quaternion rotates a vector in the NED frame to align with a vector in the body frame. To express an NED vector in the body frame, use the inverse rotation.
    
    time_range = np.linspace(0, 1, 100)
    timeseries = ct.input_output_response(rigid_body, U=np.ones((6, len(time_range)))*0.5, T=time_range, X0=x0)

    import matplotlib.pyplot as plt

    # Extract the outputs from the timeseries
    outputs = timeseries.outputs

    # Plot each output
    fig, axs = plt.subplots(outputs.shape[0], 1, figsize=(10, 20))
    for i in range(outputs.shape[0]):
        axs[i].plot(time_range, outputs[i, :])
        axs[i].set_title(f'Output {i+1}')
        axs[i].set_xlabel('Time (s)')
        axs[i].set_ylabel('Value')

    plt.tight_layout()
    plt.show()
