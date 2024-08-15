import numpy as np
import control as ct
from typing import List

def aero_coefficients(mach, alpha, beta, surfaces: List):

    # Coefficients in order CFx, CFy, CFz, CMx, CMy, CMz
    # coefficients = aero_data.interpolate(mach, alpha, beta, surfaces)

    coefficients = aero_model.interpolate(alpha)
    
    # TODO: have AI write this function
    # Check for NaN values in coefficients and replace them with zero
    coefficients = [0 if np.isnan(coeff) else coeff for coeff in coefficients]

    CD, CL, CM, CN, CA, XCP, CLA, CMA, CYB, CNB, CLB = coefficients

    # In DATCOM Figure 5, +Z points up and +X points aft
    CZ = -CN # normal force coefficient from datcom points opposite body-fixed frame
    CX = -CA # axial force coefficient from datcom points opposite body-fixed frame

    CY = CYB*beta
    CMx = CLB*beta
    CMy = CM
    CMz = CNB*beta

    # rot = np.array([[np.cos(alpha), 0, -np.sin(alpha)], [0, 1, 0], [np.sin(alpha), 0, np.cos(alpha)]])
    # rotate
    # return np.zeros(3), np.zeros(3)
    return np.array([CX, CY, CZ]), np.array([CMx, CMy, CMz])
    
def aero_forces_moments(CF, CM, airspeed, params):
    # Calculate the aerodynamic forces and moments based on the provided coefficients and flight conditions
    # CF: Coefficient of Forces, a vector [CFx, CFy, CFz]
    # CM: Coefficient of Moments, a vector [CMx, CMy, CMz]

    rho, S_ref, b_ref, c_ref, r_ref, r_cg = params['density'], params['S_ref'], params['b_ref'], params['c_ref'], params['r_ref'], params['r_cg']
    
    q = .5 * rho * airspeed**2

    F = q * S_ref * CF

    # Calculate moments
    M = q * S_ref * np.array([b_ref * CM[0], c_ref * CM[1], b_ref * CM[2]])

    # Transfer moment from the datum to the center of gravity
    M += np.cross(r_ref - r_cg, F)

    return F, M

def calc_aerodynamics_outputs(t, x, u, params):

    airspeed = u[0]
    alpha = u[1]
    beta = u[2]

    CF, CM = aero_coefficients(0, alpha, beta, [0, 0, 0])

    return aero_forces_moments(CF, CM, airspeed, params)


aerodynamics_state_inputs = ['airspeed', 'alpha', 'beta']
aerodynamics_control_inputs = ['deflections']
aerodynamics_inputs = aerodynamics_state_inputs + aerodynamics_control_inputs
aerodynamics_outputs = ['Fx_aero', 'Fy_aero', 'Fz_aero', 'Mx_aero', 'My_aero', 'Mz_aero']
aerodynamics = ct.nlsys(updfcn=None, outfcn=calc_aerodynamics_outputs, inputs=aerodynamics_inputs, outputs=aerodynamics_outputs, name='aerodynamics')

if __name__ == "__main__":
    time_range = np.linspace(0, 10, 100)
    from .parameters import parameters
    aerodynamics.params = parameters
    timeseries = ct.input_output_response(aerodynamics, T=time_range, U=np.ones([2, len(time_range)])*0.5)