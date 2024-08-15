import pandas as pd
import os

from utilities.interpolation import Interpolator

current_dir = os.path.dirname(os.path.abspath(__file__))
mass_data_path = os.path.join(current_dir, '..', 'data', 'mass_table.csv')
mass_data = pd.read_csv(mass_data_path)

# Inertia should be expressed in the body-fixed frame located at the c.g. (further processing will be needed for other conventions)
mass_lookup = Interpolator(1, mass_data_path)

def get_mass_properties(fuel_level):
    # all units SI

    if not isinstance(fuel_level, list):
        fuel_level = [fuel_level]
    
    interpolated_values = mass_lookup.interpolate([fuel_level])
    mass = interpolated_values[0]
    cg_location = interpolated_values[1:4]
    Ixx, Iyy, Izz, Ixy, Ixz, Iyz  = interpolated_values[4:10]
    inertia_matrix = [
        [Ixx, -Ixy, -Ixz],
        [-Ixy, Iyy, -Iyz],
        [-Ixz, -Iyz, Izz]
    ]
    inertia = inertia_matrix
    return mass, cg_location, inertia


