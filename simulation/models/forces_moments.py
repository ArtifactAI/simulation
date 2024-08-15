import control as ct
import numpy as np

from .aerodynamics import aerodynamics, aerodynamics_inputs, aerodynamics_outputs
from .propulsion import propulsion, propulsion_inputs, propulsion_outputs
from .gravity import gravity, gravity_inputs, gravity_outputs

def summation(t, x, u, params):

    aero_forces = u[:3]
    aero_moments = u[3:6]
    prop_forces = u[6:9]
    prop_moments = u[9:12]
    gravity_forces = u[12:15]

    forces = aero_forces + prop_forces + gravity_forces
    moments = aero_moments + prop_moments # Expects moments expressed in the body-fixed frame located at the c.g.
    
    return forces, moments

sum_forces_moments_inputs = aerodynamics_outputs + propulsion_outputs + gravity_outputs
sum_forces_moments_outputs = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
sum_forces_moments = ct.nlsys(updfcn=None, outfcn=summation, inputs=sum_forces_moments_inputs, outputs=sum_forces_moments_outputs, name='sum_forces_moments')

forces_moments_inputs = aerodynamics_inputs + propulsion_inputs + gravity_inputs
forces_moments_outputs = aerodynamics_outputs + propulsion_outputs + gravity_outputs + sum_forces_moments_outputs

forces_moments = ct.interconnect(
    (sum_forces_moments, aerodynamics, propulsion, gravity), # order matters for systems with states
    name="forces_moments",
    inplist = forces_moments_inputs, inputs = forces_moments_inputs,
    outlist = forces_moments_outputs, outputs = forces_moments_outputs
)

# test summation
if __name__ == "__main__":
    timeseries = ct.input_output_response(forces_moments, T=np.linspace(0, 1, 100))
