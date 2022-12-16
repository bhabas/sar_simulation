import numpy as np

"""This script calculates the pose values for the leg design and pad to be placed in the model.sdf file for the leg design.
"""

L = 75e-3                   # Leg length [m]
Psi = np.radians(30)        # Leg angle [deg]
theta = np.radians(45+90)   # Yaw rotation [deg]

X_offset = 29.29e-3         # Attachment x_offset [m]
Z_offset = -3.25e-3         # Attachment z_offset [m]

## CALC LEG POSE
X_Leg = X_offset*np.cos(theta)
Y_Leg = X_offset*np.sin(theta)
print(f"Leg Inertial Pose: 0 0 -{L/2:2} 0 0 0")
print(f"Leg Pose: {X_Leg:.2E} {Y_Leg:.2E} {Z_offset:.2E} 0 {-Psi:.3f} {theta:.3f}")

## CALC PAD POSE
X_pad_local = L*np.sin(Psi) + X_offset
Y_pad_local = 0
Z_pad_local = -L*np.cos(Psi) + Z_offset
Pad_Pose_Local = np.array([[X_pad_local,Y_pad_local,Z_pad_local]]).T

# YAW ROTATION MATRIX
R_z = np.array((
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta),  np.cos(theta), 0],
    [0,              0,             1]))

# CONVERT TO BODY-CENTRIC COORDS
Pad_Pose_Body = np.dot(R_z,Pad_Pose_Local).flatten()
print(f"Pad Pose: {Pad_Pose_Body[0]:.2E} {Pad_Pose_Body[1]:.2E} {Pad_Pose_Body[2]:.2E} 0 0 {theta:.3f}")

