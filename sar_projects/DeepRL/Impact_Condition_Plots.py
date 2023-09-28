import matplotlib.pyplot as plt
import numpy as np


Plane_Angle = 0

gamma = 45
L = 150e-3
PD = 75e-3

a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(90-gamma))
Beta_min = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))