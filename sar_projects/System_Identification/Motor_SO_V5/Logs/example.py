import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def piecewise_func(x, x0, a, b, c, d):
    """
    Piecewise function that is quadratic for x < x0 and linear for x >= x0.
    Parameters:
    - x0: The breakpoint between quadratic and linear
    - a, b, c: Parameters for the quadratic part (ax^2 + bx + c)
    - d: Slope for the linear part (dx)
    """
    return np.piecewise(x, [x < x0, x >= x0],
                        [lambda x: a * x**2 + b * x + c, 
                         lambda x: d * x + (a * x0**2 + b * x0 + c - d * x0)])

# Sample data
x_data = np.linspace(0, 10, 100)
y_data = piecewise_func(x_data, 5, 1, -2, 3, 0.5) + np.random.normal(0, 1, x_data.size) # Adding some noise

# Curve fitting
params, _ = curve_fit(piecewise_func, x_data, y_data)

# Plotting
plt.figure(figsize=(8, 5))
plt.scatter(x_data, y_data, label='Data')
plt.plot(x_data, piecewise_func(x_data, *params), label='Fit', color='red')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Piecewise Function Fit')
plt.legend()
plt.show()

print(f"Fit Parameters: {params}")
