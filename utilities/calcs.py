import numpy as np

def moment_transfer(moment, force, r_2, r_1 = np.zeros(3)):
    # express a moment at point r_1 about r_2 given the moment vector and force vector
    return moment +  np.cross(r_1 - r_2, force)