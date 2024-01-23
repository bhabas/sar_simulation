import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation

# Constants
L1, L2 = 1.0, 1.0  # Lengths of the pendulum arms
m1, m2 = 1.0, 1.0  # Masses of the pendulum bobs
g = 9.81  # Acceleration due to gravity

# Initial conditions
theta1, theta2 = np.pi/2, np.pi/2  # Initial angles
omega1, omega2 = 0.0, 0.0  # Initial angular velocities

# Time settings
T = 20    # Total time
dt = 0.05  # Time step
times = np.arange(0, T, dt)

def derivatives(t, state):
    theta1, omega1, theta2, omega2 = state
    dtheta1 = omega1
    dtheta2 = omega2

    delta = theta2 - theta1
    den1 = (m1 + m2) * L1 - m2 * L1 * np.cos(delta) ** 2
    den2 = (L2/L1) * den1

    domega1 = ((m2 * g * np.sin(theta2) * np.cos(delta) +
                m2 * L2 * omega2 ** 2 * np.sin(delta) +
                m2 * L1 * omega1 ** 2 * np.sin(delta) * np.cos(delta)) -
                (m1 + m2) * g * np.sin(theta1)) / den1

    domega2 = ((-m2 * L2 * omega2 ** 2 * np.sin(delta) * np.cos(delta) +
                (m1 + m2) * g * np.sin(theta1) * np.cos(delta) -
                (m1 + m2) * L1 * omega1 ** 2 * np.sin(delta)) -
                (m1 + m2) * g * np.sin(theta2)) / den2

    return [dtheta1, domega1, dtheta2, domega2]

# Initial state
state0 = [theta1, omega1, theta2, omega2]

# Solve the ODE
solution = solve_ivp(derivatives, [0, T], state0, t_eval=times, method='RK45')

# Setup figure and axes
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Set up the pendulum plot
ax1.set_xlim(-2 * (L1 + L2), 2 * (L1 + L2))
ax1.set_ylim(-2 * (L1 + L2), 2 * (L1 + L2))
line, = ax1.plot([], [], 'o-', lw=2)
point, = ax1.plot([], [], 'ro', markersize=8)

# Set up the theta plot
ax2.set_xlim(0, T)
ax2.set_ylim(-np.pi, np.pi)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle (rad)')

theta1_line, = ax2.plot(times, solution.y[0], '--',lw=2,label=r'$\theta_1$',color="tab:blue",alpha=0.4)
theta2_line, = ax2.plot(times, solution.y[2], '--',lw=2,label=r'$\theta_2$',color="tab:orange",alpha=0.4)

theta1_dot, = ax2.plot([], [], 'o', markersize=8, label=r'$\theta_1$',color="tab:blue")
theta2_dot, = ax2.plot([], [], 'o', markersize=8,label=r'$\theta_2$',color="tab:orange")
ax2.legend()

def update(frame, line, point, theta1_line, theta2_line):
    state = solution.y[:, frame]
    
    # Update pendulum position
    x1, y1 = L1 * np.sin(state[0]), -L1 * np.cos(state[0])
    x2, y2 = x1 + L2 * np.sin(state[2]), y1 - L2 * np.cos(state[2])
    line.set_data([0, x1, x2], [0, y1, y2])
    point.set_data([x1/2, x2/2], [y1/2, y2/2])

    # Update theta plots
    theta1_line.set_data(times[frame], solution.y[0, frame])
    theta2_line.set_data(times[frame], solution.y[2, frame])

    return line, point, theta1_line, theta2_line

anim = FuncAnimation(fig, update, frames=len(times), fargs=(line, point, theta1_dot, theta2_dot), interval=dt*1000, blit=True)

plt.tight_layout()
plt.show()
