import numpy as np
import control as ct
import matplotlib.pyplot as plt  # Add this import

from .models.vehicle import vehicle
from .configuration import parameters
from analysis.trim import solve_trim

# Simulation parameters
time = 30
dt = 0.1  # Specify the time step

throttle = 3
vehicle.params = parameters

trim_cond = solve_trim(vehicle, throttle)
print('Trim V: ', trim_cond[0])
print('Trim Alpha: ', np.degrees(trim_cond[1]))
print('Trim Theta: ', np.degrees(trim_cond[2]))

V_trim = trim_cond[0]
alpha_trim = trim_cond[1]
theta_trim = trim_cond[2] + .05

q0 = np.cos(theta_trim / 2)
q1 = 0  # No roll
q2 = np.sin(theta_trim / 2)  # Pitch
q3 = 0  # No yaw

u0 = V_trim*np.cos(alpha_trim)
w0 = V_trim*np.sin(alpha_trim)

initial_state = {
    'x': 0.0,  # Initial x position
    'y': 0.0,  # Initial y position
    'z': -1000,  # Initial z position
    'u': u0,  # Initial body-frame x-axis velocity
    'v': 0.0,  # Initial body-frame y-axis velocity
    'w': w0,  # Initial body-frame z-axis velocity
    'q0': q0, # Initial quaternion component (real part)
    'q1': q1, # Initial quaternion component (i)
    'q2': q2, # Initial quaternion component (j)
    'q3': q3, # Initial quaternion component (k)
    'p': 0.0,  # Initial roll rate
    'q': 0.0,  # Initial pitch rate
    'r': 0.0,   # Initial yaw rate
    'prop_FL': 0, # Initial propeller FL
    'prop_FR': 0, # Initial propeller FR
    'prop_BL': 0, # Initial propeller BL
    'prop_BR': 0, # Initial propeller BR
    'prop_P': throttle, # Initial propeller P
}

x0 = np.array([initial_state[key] for key in [
    'x', 'y', 'z', 'u', 'v', 'w', 'q0', 'q1', 'q2', 'q3', 'p', 'q', 'r', 'prop_FL', 'prop_FR', 'prop_BL', 'prop_BR', 'prop_P'
]])

time_range = np.arange(0, time + dt, dt)

timeseries = ct.input_output_response(vehicle, T=time_range, X0=x0, U=throttle)

u = timeseries.inputs
y = timeseries.outputs
x = timeseries.states
t = timeseries.time

# Plot Air Data
fig, ax1 = plt.subplots()
line1, = ax1.plot(t, y[vehicle.find_output('airspeed')], label='airspeed')
ax2 = ax1.twinx()
line2, = ax2.plot(t, np.degrees(y[vehicle.find_output('alpha')]), label='alpha', color='red')
line3, = ax2.plot(t, np.degrees(y[vehicle.find_output('beta')]), label='beta')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('m/s')
ax2.set_ylabel('degrees')
plt.title('Air Data')
lines = [line1, line2, line3]
labels = [line.get_label() for line in lines]
ax1.legend(lines, labels)

# Plot 3D Vehicle Position
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

x_pos = y[vehicle.find_output('x')]
y_pos = y[vehicle.find_output('y')]
altitude = -y[vehicle.find_output('z')]

ax.plot(x_pos, y_pos, altitude)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Altitude (m)')
ax.set_title('3D Vehicle Position')

# Add a colorbar to show time progression
scatter = ax.scatter(x_pos, y_pos, altitude, c=t, cmap='viridis')
cbar = fig.colorbar(scatter, ax=ax, label='Time (s)')

# Plot Euler Angles
fig, ax = plt.subplots()
phi = np.degrees(y[vehicle.find_output('phi')])
theta = np.degrees(y[vehicle.find_output('theta')])
psi = np.degrees(y[vehicle.find_output('psi')])

ax.plot(t, phi, label='Roll (φ)')
ax.plot(t, theta, label='Pitch (θ)')
ax.plot(t, psi, label='Yaw (ψ)')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (degrees)')
ax.set_title('Euler Angles')
ax.legend()
plt.grid(True)

# Plot Aerodynamic, Propulsion, and Gravity Forces
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15), sharex=True)

# Aerodynamic Forces
Fx_aero = y[vehicle.find_output('Fx_aero')]
Fy_aero = y[vehicle.find_output('Fy_aero')]
Fz_aero = y[vehicle.find_output('Fz_aero')]

ax1.plot(t, Fx_aero, label='Fx_aero')
ax1.plot(t, Fy_aero, label='Fy_aero')
ax1.plot(t, Fz_aero, label='Fz_aero')
ax1.set_ylabel('Force (N)')
ax1.set_title('Aerodynamic Forces')
ax1.legend()
ax1.grid(True)

# Propulsion Forces
Fx_prop = y[vehicle.find_output('Fx_prop')]
Fy_prop = y[vehicle.find_output('Fy_prop')]
Fz_prop = y[vehicle.find_output('Fz_prop')]

ax2.plot(t, Fx_prop, label='Fx_prop')
ax2.plot(t, Fy_prop, label='Fy_prop')
ax2.plot(t, Fz_prop, label='Fz_prop')
ax2.set_ylabel('Force (N)')
ax2.set_title('Propulsion Forces')
ax2.legend()
ax2.grid(True)

# Gravity Force
Fx_grav = y[vehicle.find_output('Fx_grav')]
Fy_grav = y[vehicle.find_output('Fy_grav')]
Fz_grav = y[vehicle.find_output('Fz_grav')]
# F_grav_norm = np.sqrt(Fx_grav**2 + Fy_grav**2 + Fz_grav**2)

ax3.plot(t, Fx_grav, label='Fx_grav')
ax3.plot(t, Fy_grav, label='Fy_grav')
ax3.plot(t, Fz_grav, label='Fz_grav')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Force (N)')
ax3.set_title('Gravity Forces')
ax3.legend()
ax3.grid(True)

# Plot Angular Rates
fig, ax = plt.subplots(figsize=(10, 5))

p = y[vehicle.find_output('p')]
q = y[vehicle.find_output('q')]
r = y[vehicle.find_output('r')]

ax.plot(t, p, label='p (roll rate)')
ax.plot(t, q, label='q (pitch rate)')
ax.plot(t, r, label='r (yaw rate)')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Angular Rate (rad/s)')
ax.set_title('Angular Rates')
ax.legend()
ax.grid(True)

# Plot Velocity Components
fig, ax = plt.subplots(figsize=(10, 5))

u = y[vehicle.find_output('u')]
v = y[vehicle.find_output('v')]
w = y[vehicle.find_output('w')]

ax.plot(t, u, label='u (forward velocity)')
ax.plot(t, v, label='v (lateral velocity)')
ax.plot(t, w, label='w (vertical velocity)')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (m/s)')
ax.set_title('Velocity Components')
ax.legend()
ax.grid(True)

# Plot Angle of Attack
fig, ax = plt.subplots(figsize=(10, 5))

w = y[vehicle.find_output('w')]
u = y[vehicle.find_output('u')]

alpha = np.arctan2(w, u)

ax.plot(t, alpha, label='Angle of Attack')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (rad)')
ax.set_title('Angle of Attack')
ax.legend()
ax.grid(True)

# Plot Overall Forces
fig, ax = plt.subplots(figsize=(10, 5))

Fx = y[vehicle.find_output('Fx')]
Fy = y[vehicle.find_output('Fy')]
Fz = y[vehicle.find_output('Fz')]

ax.plot(t, Fx, label='Fx')
ax.plot(t, Fy, label='Fy')
ax.plot(t, Fz, label='Fz')

ax.set_xlabel('Time (s)')
ax.set_ylabel('Force (N)')
ax.set_title('Overall Forces')
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.show()


