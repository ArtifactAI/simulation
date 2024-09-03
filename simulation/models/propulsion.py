import control as ct
import numpy as np
from scipy.spatial.transform import Rotation as R

from ..configuration import propulsion_controls, parameters

def calc_propulsion_dynamics(t, x, u, params):
    # u = throttle
    xdot  = np.array([(1/thruster.time_constant) * (-x[index] + u[index]) for index, thruster in enumerate(params['thrusters'])])
    return xdot

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
propulsion_control_inputs = propulsion_controls
propulsion_inputs = propulsion_state_inputs + propulsion_control_inputs
propulsion_outputs=['Fx_prop', 'Fy_prop', 'Fz_prop', 'Mx_prop', 'My_prop', 'Mz_prop']

num_engines = len(parameters['thrusters'])
propulsion = ct.nlsys(updfcn=calc_propulsion_dynamics, outfcn=calc_propulsion_outputs, inputs=propulsion_inputs, outputs=propulsion_outputs, states=num_engines, name='propulsion')

#
# test propulsion subsystem
# timeseries = ct.input_output_response(propulsion, T=np.linspace(0, 1, 100))

# if __name__ == "__main__":
#     from .parameters import parameters
