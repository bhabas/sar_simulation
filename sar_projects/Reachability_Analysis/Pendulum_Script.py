import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

class Pendulum:
    def __init__(self, length=1, g=9.81):
        self.length = length
        self.g = g

    def equation(self, t, y):
        theta, omega = y
        dtheta_dt = omega
        domega_dt = -(self.g / self.length) * np.sin(theta)
        return [dtheta_dt, domega_dt]

    def solve(self, y0, t_span, dt):
        t = np.arange(t_span[0], t_span[1], dt)
        sol = solve_ivp(self.equation, [t[0], t[-1]], y0, t_eval=t)
        return sol.t, sol.y

class PendulumAnimation:
    def __init__(self, pendulum, y0, t_span, dt):
        self.pendulum = pendulum
        self.t, self.y = pendulum.solve(y0, t_span, dt)
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'o-', lw=2)
        self.ax.set_xlim(-pendulum.length*1.2, pendulum.length*1.2)
        self.ax.set_ylim(-pendulum.length*1.2, pendulum.length*1.2)
        self.ax.set_aspect('equal', 'box')

    def init(self):
        self.line.set_data([], [])
        return self.line,

    def update(self, i):
        x = self.pendulum.length * np.sin(self.y[0, i])
        y = -self.pendulum.length * np.cos(self.y[0, i])
        self.line.set_data([0, x], [0, y])
        return self.line,

    def animate(self):
        ani = FuncAnimation(self.fig, self.update, frames=len(self.t), init_func=self.init, blit=True, interval=20)
        plt.show()

# Usage
pendulum = Pendulum(length=1, g=9.81)
pendulum_animation = PendulumAnimation(pendulum, y0=[np.pi/4, 0], t_span=[0, 10], dt=0.05)
pendulum_animation.animate()
