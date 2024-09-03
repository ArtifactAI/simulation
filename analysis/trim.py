import numpy as np
from scipy.optimize import fsolve
from scipy.optimize import minimize

# TODO: make generic trim solver

def solve_trim(system, initial_condition):

    V = initial_condition['airspeed']
    z0 = -initial_condition['altitude']

    def cost_function(vars):
        alpha = vars[0]
        throttle = vars[1]
        elevator = vars[2]
        ailerons = 0

        u = V*np.cos(alpha)
        w = V*np.sin(alpha)
        theta = alpha # assume level flight

        # Calculate quaternion from pitch angle (theta)
        q0 = np.cos(theta / 2)
        q1 = 0  # No roll
        q2 = np.sin(theta / 2)  # Pitch
        q3 = 0  # No yaw

        x = np.array([0, 0, z0, u, 0, w, q0, q1, q2, q3, 0, 0, 0, throttle])
        # x = np.array([0, 0, -1000, 100, 0, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, throttle])
        xdot = system.dynamics(0, x, [elevator, ailerons, throttle], system.params)
        
        udot = xdot[3]
        wdot = xdot[5]
        qdot = xdot[11]

        return udot**2 + wdot**2 + qdot**2
    
    bounds = [(np.radians(-2), np.radians(14)), # Bounds for alpha
              (0, 1),  # Bounds for throttle
              (np.radians(-29), np.radians(24))]  # Bounds for elevator
    initial_guess = [np.radians(5), .7, np.radians(0)]
    result = minimize(cost_function, initial_guess, method='L-BFGS-B', bounds=bounds, tol=1e-6)

# TODO: evaluate quality of trim solution
