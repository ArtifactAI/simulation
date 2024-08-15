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

# Setup Propellers
# Assume the propeller frame is constructed to orient thrust in the positive x-direction
lift_thruster_rotation = R.from_euler('y', 90, degrees=True) # rotates propeller-frame thrust vector (thrust in x-direction) in body frame
pusher_rotation = R.identity() # pusher prop thrust is aligned with body x-axis

prop_front_left = Thruster('prop_front_left', np.array([0, 0, 0]), lift_thruster_rotation, .1, 100)
prop_front_right = Thruster('prop_front_right', np.array([0, 0, 0]), lift_thruster_rotation, .1, 100)
prop_rear_left = Thruster('prop_rear_left', np.array([0, 0, 0]), lift_thruster_rotation, .1, 100)
prop_rear_right = Thruster('prop_rear_right', np.array([0, 0, 0]), lift_thruster_rotation, .1, 100)
prop_pusher = Thruster('prop_pusher', np.array([-1, 0, 0]), pusher_rotation, .1, 100)

num_engines = 5