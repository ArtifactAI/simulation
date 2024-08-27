import numpy as np
import control as ct
from typing import List

from ..configuration import aerodynamics_model
from ..configuration import aero_controls

surfaces_dict = {}

def calc_aerodynamics_outputs(t, x, u, params):

    airspeed = u[0]
    alpha = u[1]
    beta = u[2]
    p = u[3]
    q = u[4]
    r = u[5]
    surfaces = u[6:]

    for i, control in enumerate(aero_controls):
        surfaces_dict[control] = surfaces[i]
        
    F, M = aerodynamics_model(airspeed, alpha, beta, p, q, r, surfaces_dict, params)

    return F, M


aerodynamics_state_inputs = ['airspeed', 'alpha', 'beta', 'p', 'q', 'r']
aerodynamics_control_inputs = aero_controls
aerodynamics_inputs = aerodynamics_state_inputs + aerodynamics_control_inputs
aerodynamics_outputs = ['Fx_aero', 'Fy_aero', 'Fz_aero', 'Mx_aero', 'My_aero', 'Mz_aero']
aerodynamics = ct.nlsys(updfcn=None, outfcn=calc_aerodynamics_outputs, inputs=aerodynamics_inputs, outputs=aerodynamics_outputs, name='aerodynamics')