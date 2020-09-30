import numpy as np
import rigidmotion as rm

class GeometricController:
    def __init__(self, m, J: np.mat, c_T, c_tau, d, k: np.ndarray):
        self.m, self.J, self.g = m, J, 9.8

        self.c_T = c_T
        gamma = np.mat( [[1,1,1,1], [-d,d,d,-d], [-d,d,-d,d], [-c_tau,-c_tau,c_tau,c_tau]] )
        self.gamma_inv = gamma.I

        self.k = k

    def rotor_speed(self, state: np.mat, command: np.mat):
        m, J, g = self.m, self.J, self.g
        k_x, k_v, k_R, k_omega = self.k 

        position = state[0:3]
        orientation_q = state[3:7]
        vel = state[7:10]
        omega = state[10:13]
        R = rm.quat2rotm(quat=orientation_q)

        control_type = command[0,0]     # 1:position, 2:velocity, 3:attitude, 4:angular rate control
        if control_type==1 or control_type==2:      
            if control_type==1:         # position control
                p_d = command[1:4]
                e_x = position - p_d
                e_v = vel - np.mat("0;0;0")

                e_x[0,0] = np.sign(e_x[0,0])*0.5 if np.abs(e_x[0,0])>0.5 else e_x[0,0]
                e_x[1,0] = np.sign(e_x[1,0])*0.5 if np.abs(e_x[1,0])>0.5 else e_x[1,0]
            else:                       # velocity control
                v_d = command[1:4]
                e_x = np.mat("0;0;0")
                e_v = vel - v_d

            b3_d = -k_x*e_x - k_v*e_v + m*np.mat([[0],[0],[g]])
            b3_d[2,0] = 1e-2 if b3_d[2,0] <= 0 else b3_d[2,0]
            if np.linalg.norm(b3_d[0:2,0]) > np.abs(b3_d[2,0]):         # limit max pitch angle to 45 deg
                b3_d[2,0] = np.linalg.norm(b3_d[0:2,0])
            b3_d = b3_d / np.linalg.norm(b3_d)
            b1_d = np.mat("1;0;0")
            b2_d = rm.hat(b3_d)*b1_d              # b3_d x b1_d
            b2_d = b2_d / np.linalg.norm(b2_d)
            b1_d = rm.hat(b2_d)*b3_d              # b2_d x b3_d
            R_d = np.hstack([b1_d, b2_d, b3_d])

            e_R = 0.5 * rm.dehat( R_d.T*R - R.T*R_d )
            e_omega = omega - np.mat("0;0;0")

            f = (-k_x*e_x -k_v*e_v + m*np.mat([[0],[0],[g]]) ).T * R[:,2]
            M = -k_R*e_R -k_omega*e_omega + rm.hat(omega)*J*omega

        elif control_type==3 or control_type==4:
            if control_type == 3:       # attitude control
                R_d = rm.eul2rotm(eul_angles=command[1:4], sequence="ZYX")
                e_R = 0.5 * rm.dehat( R_d.T*R - R.T*R_d )
                e_omega = omega - np.mat("0;0;0")
            else:
                omega_d = command[1:4]
                e_R = np.mat("0;0;0")
                e_omega = omega - omega_d

            f = m*1.07*g / R[2,2] if R[2,2]>0.705 else m*1.07*g/0.705
            M = -k_R*e_R -k_omega*e_omega + rm.hat(omega)*J*omega

        f_rotors = self.gamma_inv * np.vstack([f,M])
        f_rotors[f_rotors<0]=0
        rotor_speed = np.sqrt( f_rotors / self.c_T )

        return np.array(rotor_speed).reshape(1,-1)[0,:]

    



