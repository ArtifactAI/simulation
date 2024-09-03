import numpy as np
from scipy.spatial.transform import Rotation as R

class Thruster:
    def __init__(self, name, position, rotation, time_constant, max_thrust):
        # orientation
        self.name = name
        self.position = position # vector relative to vehicle datum
        self.rotation = rotation # rotation from body frame to thrust frame (expresses a thrust-frame vector in the body frame)
        self.time_constant = time_constant
        self.max_thrust = max_thrust # Newtons
        # add force map
        # add rotor inertia

# Assume the propeller frame is constructed to orient thrust in the positive x-direction
vertically_alighed_engine = R.from_euler('y', 90, degrees=True) # rotates propeller-frame thrust vector (thrust in x-direction) in body frame
horizontal_engine = R.identity() # pusher prop thrust is aligned with body x-axis
