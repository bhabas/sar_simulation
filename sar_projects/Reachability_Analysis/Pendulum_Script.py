import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from matplotlib.animation import FuncAnimation

# Constants
L1, L2 = 1.0, 1.0  # Lengths of the pendulum arms
m1, m2 = 1.0, 1.0  # Masses of the pendulum bobs
g = 9.81  # Acceleration due to gravity

J_a = 1/3*m1*L1**2 + m2*L1**2
J_b = 1/3*m2*L2**2
J_x = 1/2*m2*L1*L2

mu_1 = (1/2*m1 + m2)*g*L1
mu_2 = 1/2*m2*g*L2

Kt = 50
Kt_theta = np.pi/2

# Initial conditions
theta1, theta2 = 0.1, np.pi/2  # Initial angles
omega1, omega2 = 0.0, 0.0  # Initial angular velocities

# Time settings
T = 20    # Total time
dt = 0.05  # Time step
times = np.arange(0, T, dt)

def derivatives(t, state):
    theta1, omega1, theta2, omega2 = state
    dtheta1 = omega1
    dtheta2 = omega2

    # Matrix A for the linear system Ax = b
    A = np.array([[J_a, J_x*np.cos(theta1-theta2)],
                  [J_x*np.cos(theta1-theta2), J_b]])

    # Vector b
    b = np.array([-J_x/J_a*np.sin(theta1-theta2)*dtheta2**2 - mu_1*np.sin(theta1) + Kt*((theta2-Kt_theta) - theta1),
                  J_x/J_b*np.sin(theta1-theta2)*dtheta1**2 - mu_2*np.sin(theta2) - Kt*((theta2-Kt_theta) - theta1)])

    # Solve the linear system
    domega1, domega2 = np.linalg.solve(A, b)

    return [dtheta1, domega1, dtheta2, domega2]


# Initial state
state0 = [theta1, omega1, theta2, omega2]

# Solve the ODE
solution = solve_ivp(derivatives, [0, T], state0, t_eval=times, method='RK45')

# Setup figure and axes
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

# Set up the pendulum plot
ax1.set_xlim(-2 * (L1 + L2), 2 * (L1 + L2))
ax1.set_ylim(-2 * (L1 + L2), 0.5)
ax1.axhline(0,-5,5,lw=2,color="k",alpha=0.4)
ax1.set_aspect('equal', adjustable='box')
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

def update(frame, line, CoM_point, theta1_dot, theta2_dot):
    state = solution.y[:, frame]
    
    # Update pendulum position
    x1, y1 = L1 * np.sin(state[0]), -L1 * np.cos(state[0])
    x2, y2 = x1 + L2 * np.sin(state[2]), y1 - L2 * np.cos(state[2])
    line.set_data([0, x1, x2], [0, y1, y2])

    # Corrected CoM position for second pendulum
    x1_com, y1_com = x1 / 2, y1 / 2  # CoM of first pendulum
    x2_com, y2_com = (x1 + x2) / 2, (y1 + y2) / 2  # CoM of second pendulum
    CoM_point.set_data([x1_com, x2_com], [y1_com, y2_com])

    # Update theta plot dots
    theta1_dot.set_data([times[frame]], [solution.y[0, frame]])
    theta2_dot.set_data([times[frame]], [solution.y[2, frame]])

    return line, CoM_point, theta1_dot, theta2_dot

anim = FuncAnimation(fig, update, frames=len(times), fargs=(line, point, theta1_dot, theta2_dot), interval=dt*1000, blit=True)

plt.tight_layout()
plt.show(block=True)
