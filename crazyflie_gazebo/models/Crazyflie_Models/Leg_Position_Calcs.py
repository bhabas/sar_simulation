import numpy as np

"""This script calculates the pose values for the leg design and pad to be placed in the model.sdf file for the leg design.
"""

L = 100      # Leg length [mm]
Psi = 5     # Leg angle [deg]


L=L*1e-3            # Convert to meters
X_offset = 29.29e-3 # Attachment x_offset [m]
Z_offset = -3.25e-3 # Attachment z_offset [m]


for Theta in [45,135,-135,-45]:

    Theta_rad = np.radians(Theta)
    Psi_rad = np.radians(Psi)

    ## CALC LEG POSE
    X_Leg = X_offset*np.cos(Theta_rad)
    Y_Leg = X_offset*np.sin(Theta_rad)

    ## CALC PAD POSE
    X_pad_local = L*np.sin(Psi_rad) + X_offset + 4.5e-3
    Y_pad_local = 0
    Z_pad_local = -L*np.cos(Psi_rad) + Z_offset
    Pad_Pose_Local = np.array([[X_pad_local,Y_pad_local,Z_pad_local]]).T

    # YAW ROTATION MATRIX
    R_z = np.array((
        [np.cos(Theta_rad), -np.sin(Theta_rad), 0],
        [np.sin(Theta_rad),  np.cos(Theta_rad), 0],
        [0,              0,             1]))

    # CONVERT TO BODY-CENTRIC COORDS
    Pad_Pose_Body = np.dot(R_z,Pad_Pose_Local).flatten()

    print()
    print(f"Theta Angle: {np.degrees(Theta_rad):.2f}")
    print(f"Leg Inertial Pose: <pose>0 0 -{L/2:2} 0 0 0 </pose>")
    print(f"Leg Pose: <pose>{X_Leg:.2E} {Y_Leg:.2E} {Z_offset:.2E} 0 {-Psi_rad:.3f} {Theta_rad:.3f}</pose>")
    print(f"Pad Pose: <pose>{Pad_Pose_Body[0]:.2E} {Pad_Pose_Body[1]:.2E} {Pad_Pose_Body[2]:.2E} 0 0 {Theta_rad:.3f}</pose>")

