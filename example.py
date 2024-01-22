import numpy as np
from scipy.optimize import fsolve

m = 693.0e-3  # mass in kg
g = 9.81      # acceleration due to gravity in m/s^2
L = 18.5e-3       # length in meters
K = 24*2        # torsional spring constant in Nm/rad

# Define the equation m*g*L*sin(theta) = K*theta
def equation(theta):
    return m * g * L * np.sin(theta) - 1/2*K * theta**2

# Initial guess for theta
theta_guess = 1.0

# Solve the equation
theta_solution = fsolve(equation, theta_guess)

print(theta_solution/2)
