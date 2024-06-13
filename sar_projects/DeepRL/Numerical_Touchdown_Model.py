## IMPORT ENVIRONMENTS
from Envs.SAR_2D_DeepRL import SAR_2D_Env

## STANDARD IMPORTS
import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))


if __name__ == '__main__':

    Plane_Angle = 45
    V_mag = 1.5
    V_angle = 90

    env = SAR_2D_Env(Ang_Acc_range=[-60,0],V_mag_range=[V_mag,V_mag],V_angle_range=[V_angle,V_angle],Plane_Angle_range=[Plane_Angle,Plane_Angle],Render=True,Fine_Tune=False)

    theta_L = np.radians(90-env.Gamma_eff)
    env.Plane_Angle_deg = Plane_Angle
    env.Plane_Angle_rad = np.radians(Plane_Angle)

    # Initial conditions
    theta0 = 0.0   # Initial angle in radians
    D_perp_0 = 0.0    # Initial position
    V_perp = 1.7      # Initial velocity
    dtheta0 = 0.0  # Initial angular velocity


    # Initial conditions vector
    y0 = [D_perp_0, -V_perp, theta0, dtheta0]
    t_max = 0.9  # Maximum time in seconds

    # Solve the differential equations using scipy.integrate.solve_ivp
    sol = solve_ivp(env.touchdown_ODE, [0, t_max], y0, t_eval=np.linspace(0, t_max, 1000))

    # Extract the results
    time = sol.t
    D_perp_values = sol.y[0]
    dD_perp_values = sol.y[1]
    theta_values = sol.y[2]
    dtheta_values = sol.y[3]

    D_p_values = D_perp_values - env.Forward_Reach*np.sin(theta_values)
    D_L_values = D_perp_values - env.L_eff*np.sin(theta_values-theta_L)

    # Plotting using the axes method
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ax1.plot(time, D_perp_values, label='D_perp')
    ax1.plot(time, D_p_values, label='D_prop')
    ax1.plot(time, D_L_values, label='D_L')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('D_perp [m]')
    # ax1.set_ylim([-0.5, 0.1])
    ax1.set_xlim([0, t_max])
    ax1.legend()
    ax1.set_title('Position over Time')

    ax2.plot(time, theta_values, label='Angle (theta)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle/Angular Velocity')
    ax2.legend()
    ax2.set_title('Angle and Angular Velocity over Time')

    plt.tight_layout()
    plt.show(block=True)


    # a_Rot = -90
    # Tau_CR_trg = 0.25

    # for ep in range(50):

    #     obs,_ = env.reset()

    #     Done = False
    #     while not Done:

    #         action = env.action_space.sample() # obs gets passed in here
    #         action[0] = 0
    #         action[1] = env.scaleValue(a_Rot,env.Ang_Acc_range,[-1,1])
    #         if 0.0 < env.Tau_CR <= Tau_CR_trg:
    #             action[0] = 1
    #         obs,reward,terminated,truncated,_ = env.step(action)
    #         Done = terminated or truncated

    #     print(f"Episode: {ep} \t Reward: {reward:.3f} \t Reward_vec: ",end='')
    #     print(' '.join(f"{val:.2f}" for val in env.reward_vals))