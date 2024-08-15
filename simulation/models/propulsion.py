import control as ct
import numpy as np
from scipy.spatial.transform import Rotation as R

def calc_propulsion_dynamics(t, x, u, params):
    # u = throttle
    return np.array([(1/thruster.time_constant) * (-x[index] + u[index]) for index, thruster in enumerate(params['thrusters'])])

def calc_propulsion_outputs(t, x, u, params):

    # TODO: map the state to thrust vector
    thruster_force = np.zeros(3)
    thruster_moment = np.zeros(3)

    for index, thruster in enumerate(params['thrusters']):
        thrust_vector = thruster.max_thrust * thruster.rotation.apply(np.array([x[index], 0, 0]))
        thruster_force += thrust_vector
        thruster_moment += np.cross(thruster.position - params["r_cg"], thrust_vector)
        
    return thruster_force, thruster_moment

propulsion_state_inputs = []
propulsion_control_inputs=['throttle_FL', 'throttle_FR', 'throttle_BL', 'throttle_BR', 'throttle_P']
propulsion_inputs = propulsion_state_inputs + propulsion_control_inputs
propulsion_outputs=['Fx_prop', 'Fy_prop', 'Fz_prop', 'Mx_prop', 'My_prop', 'Mz_prop']

from .engines import *
engines = {
        'thrusters': [prop_front_left, prop_front_right, prop_rear_left, prop_rear_right, prop_pusher]
}

propulsion = ct.nlsys(updfcn=calc_propulsion_dynamics, outfcn=calc_propulsion_outputs, inputs=propulsion_inputs, outputs=propulsion_outputs, states=num_engines, params=engines, name='propulsion')

#
# test propulsion subsystem
# timeseries = ct.input_output_response(propulsion, T=np.linspace(0, 1, 100))

# if __name__ == "__main__":
#     from .parameters import parameters
