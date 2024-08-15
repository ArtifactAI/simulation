import control as ct
import numpy as np

from .aerodynamics import aerodynamics_control_inputs
from .propulsion import propulsion_control_inputs
from .rigid_body import rigid_body
from .forces_moments import forces_moments
from .euler_angles import euler_angles_calc, euler_angles_outputs
from .airdata import airdata_calcs, airdata_outputs
from .rigid_body import rigid_body_outputs
from .forces_moments import forces_moments_outputs

# vehicle_inputs = aerodynamics_control_inputs + propulsion_control_inputs
vehicle_inputs = ['throttle_P']
vehicle_outputs = rigid_body_outputs + forces_moments_outputs + euler_angles_outputs + airdata_outputs

vehicle = ct.interconnect(
    (rigid_body, forces_moments, euler_angles_calc, airdata_calcs), # subsystems with states are listed first
    name="vehicle",
    inplist=vehicle_inputs, inputs=vehicle_inputs,
    outlist=vehicle_outputs, outputs=vehicle_outputs
)

if __name__ == "__main__":
    time_range = np.linspace(0, 1, 100)
    timeseries = ct.input_output_response(vehicle, T=time_range, X0=x0)
    # U=np.ones(len(time_range))*0.5, X0=x0)
    # U=np.ones(len(time_range))*0.5
