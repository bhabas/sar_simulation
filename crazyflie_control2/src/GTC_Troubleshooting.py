import numpy as np
from scipy.spatial.transform import Rotation

def hat(vec):
        arr = np.zeros((3,3))
        vec = np.squeeze(vec)

        arr[0,1] = -vec[2]
        arr[0,2] =  vec[1]
        arr[1,0] =  vec[2]
        arr[1,2] = -vec[0]
        arr[2,0] = -vec[1]
        arr[2,1] =  vec[0]

        return arr

def dehat(arr):
    vec = np.array([[arr[2,1],arr[0,2],arr[1,0]]])

    return vec


## DEFINE SYSTEM CONSTANTS
m = 0.0376               # Mass [kg]
g = 9.8066              # Gravity [m/s^2]

d = 0.04                # Absolute distance from CoM to prop [m]
dp = d*np.sin(np.pi/4)  # Projected prop distance onto x-axis [m]

kf = 2.2e-8             # Thrust coefficient [N/(rad/s)^2] - Source: Forster
c_tf = 0.0061           # Thrust-Moment coefficient [Nm/N]

J = np.array([[1.65717e-5, 0, 0], # Inertia Matrix [kg*m^2]
            [0, 1.66556e-5, 0],
            [0, 0, 2.92617e-5]])

e_3 = np.array([[0,0,1]]).T # Define vertical z-axis


K = {
        "P_kp_xy": 0.5,
        "P_kd_xy": 0.3,
        "P_ki_xy": 0.0,

        "P_kp_z": 1.2,
        "P_kd_z": 0.35,
        "P_ki_z": 0.0,

        "R_kp_xy": 0.02,
        "R_kd_xy": 0.08,
        "R_ki_xy": 0.0,

        "R_kp_z": 0.003,
        "R_kd_z": 0.025,
        "R_ki_z": 0,
    }



Kp_P = np.array([[K['P_kp_xy'],K['P_kp_xy'],K['P_kp_z']]]).T
Kd_P = np.array([[K['P_kd_xy'],K['P_kd_xy'],K['P_kd_z']]]).T
Ki_P = np.array([[K['P_ki_xy'],K['P_ki_xy'],K['P_ki_z']]]).T


Kp_R = np.array([[K['R_kp_xy'],K['R_kp_xy'],K['R_kp_z']]]).T
Kd_R = np.array([[K['R_kd_xy'],K['R_kd_xy'],K['R_kd_z']]]).T
Ki_R = np.array([[K['R_ki_xy'],K['R_ki_xy'],K['R_ki_z']]]).T


FullState = {
        'x': np.array([[0.0,0.001,0.309]]).T,      # Pos. [x,y,z] - [m]
        'v': np.array([[-0.006,0.011,-0.007]]).T,      # Lin. Vel [vx,vy,vz] - [m/s]
        'w': np.array([[0.0,0.0,0.0]]).T,      # Ang. Vel [wx,wy,wz] - [rad/s]
        'eul': np.array([[0.682,0.029,-0.070]]).T,    
        'quat': np.array([[0,0,0,1]]),     # Orientation [qx,qy,qz,qw]
    }

DesiredState = {
        'x_d': np.array([[0.0,0.0,0.4]]).T,        # Desired Pos. [x,y,z] - [m]
        'v_d': np.array([[0.0,0.0,0.0]]).T,        # Desired Vel. [vx,vy,vz] - [m/s]
        'a_d': np.array([[0.0,0.0,0.0]]).T,        # Desired Acc. [ax,ay,az] - [m/s^2]

        'omega_d': np.array([[0.0,0.0,0.0]]).T,    # Desired Ang. Vel [wx,wy,wz] - rad/s
        'b1_d': np.array([[1.0,0.0,0.0]]).T        # Desired body x-axis (in world FoR) [x,y,z] - rad
    }

## REDEFINE STATE VALUES
statePos = FullState['x']
stateVel = FullState['v']
stateOmega = FullState['w']
stateQuat = FullState['quat']
stateEul = FullState['eul']

x_d = DesiredState['x_d']
v_d = DesiredState['v_d']
a_d = DesiredState['a_d']




# b1_d = DesiredState['b1_d']
b1_d = np.array([[1.,0.,0.]]).T
omega_d = DesiredState['omega_d']
domega_d = np.array([[0.,0.,0.]]).T



# ROTATION MATRIX
R = Rotation.from_euler('xyz', stateEul.T, degrees=True).as_matrix().reshape(3,3)
# R = Rotation.from_quat(stateQuat).as_matrix().reshape(3,3) # Trim off extra axis
b3 = R.dot(e_3)

# TRANSLATIONAL ERRORS AND DESIRED BODY-FIXED AXES
e_x = statePos - x_d
e_v = stateVel - v_d

P_effort = -Kp_P*e_x + -Kd_P*e_v
F_thrust_ideal = P_effort + m*g*e_3 + m*a_d

# ROTATIONAL ERRORS 

b3_d = F_thrust_ideal/np.linalg.norm(F_thrust_ideal)
b2_d = np.cross(b3_d,b1_d,axis=0).T
b2_d = b2_d.T/np.linalg.norm(b2_d)
R_d = np.hstack((np.cross(b2_d,b3_d,axis=0),b2_d,b3_d))


e_R = 0.5*dehat(R_d.T @ R - R.T @ R_d).T
e_w = stateOmega - R.T @ R_d @ omega_d




# # CONTROL EQUATIONS
R_effort = -Kp_R*e_R + -Kd_R*e_w
Gyro_dyn = np.cross(stateOmega,J @ stateOmega,axis=0)
Gyro_dyn += J @ (hat(stateOmega) @ R.T @ R_d @ omega_d - R.T @ R_d @ domega_d)


F_thrust = F_thrust_ideal.T.dot(b3)
M = R_effort + Gyro_dyn

print(F_thrust)
print(M*1e3)
