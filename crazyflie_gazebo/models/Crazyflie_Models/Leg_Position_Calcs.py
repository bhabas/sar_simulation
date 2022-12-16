import numpy as np

L = 75e-3               # Leg length [m]
Psi = np.radians(30)    # Leg angle [deg]
X_offset = 29.29e-3        # Attachment x_offset [m]
Z_offset = -3.25e-3        # Attachment z_offset [m]
theta = np.radians(45)  # Yaw rotation [deg]

## BUILD IN LOCAL COORDS
X = L*np.sin(Psi) + X_offset
Y = 0
Z = -L*np.cos(Psi) + Z_offset
Input_coords = np.array([[X,Y,Z]]).T

## YAW ROTATION MATRIX
R_z = np.array((
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1]))

## CONVERT TO BODY-CENTRIC COORDS
Output_coords = np.dot(R_z,Input_coords)

print(Output_coords)

