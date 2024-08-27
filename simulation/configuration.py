import numpy as np
import os, sys
from contextlib import contextmanager
from scipy.spatial.transform import Rotation as R

@contextmanager
def add_sys_path(path):
    original_sys_path = sys.path.copy()
    sys.path.append(path)
    try:
        yield
    finally:
        sys.path = original_sys_path

# Import aerodynamics model
path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '.artifact'))
with add_sys_path(path):
    from aero_model import aerodynamics_model
    from aero_model import configuration_data as parameters

aero_controls = ['elevator', 'ailerons']

# All data is given about c.g.
r_cg = np.array([-2, 0, 0]) # center of gravity

# Propulsion data
r_ref_propulsion = np.array([0, 0, 0])

# Other mass properties
density = 1.225
mass = 128
Ixx = 118.8
Iyy = 56.9
Izz = 172
Ixy = 0
Ixz = 1
Iyz = 0
inertia_matrix = [
    [Ixx, -Ixy, -Ixz],
    [-Ixy, Iyy, -Iyz],
    [-Ixz, -Iyz, Izz]
]

parameters.update({
    'density': density,
    'r_ref_propulsion': r_ref_propulsion,
    'r_cg': r_cg,
    'mass': mass,
    'inertia': inertia_matrix,
})