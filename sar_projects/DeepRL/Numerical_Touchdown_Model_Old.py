## IMPORT ENVIRONMENTS
from Envs.SAR_2D_DeepRL import SAR_2D_Env

## STANDARD IMPORTS
import os
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import minimize_scalar



## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))


if __name__ == '__main__':


    # Initial conditions
    Plane_Angle = 0
    theta_0 = np.radians(0+Plane_Angle)   # Initial angle in radians
    D_perp_0 = 0.0    # Initial position
    V_perp = 4.0      # Initial velocity
    dtheta0 = 0.0  # Initial angular velocity

    env = SAR_2D_Env(Ang_Acc_range=[-90,0])

    theta_Leg = np.radians(90-env.Gamma_eff)
    env.Plane_Angle_deg = Plane_Angle
    env.Plane_Angle_rad = np.radians(Plane_Angle)


    def first_min(values, theta_values):
        for i in range(1, len(values) - 1):

            if theta_values[i] >= np.pi:
                break

            if values[i] < values[i - 1] and values[i] < values[i + 1]:
                return values[i]
            
        return values[0]  # Default in case no peak is found
    
    t_max = 0.9  # Maximum time in seconds

    def objective(V_perp):

        # Initial conditions vector
        y0 = [D_perp_0, -V_perp, theta_0, dtheta0]

        # Solve the differential equations using scipy.integrate.solve_ivp
        sol = solve_ivp(env.touchdown_ODE_MotorFilter_Old, [0, t_max], y0, t_eval=np.linspace(0, t_max, 1000))

        # Extract the results
        time = sol.t
        D_perp_values = sol.y[0]
        dD_perp_values = sol.y[1]
        theta_values = sol.y[2]
        dtheta_values = sol.y[3]

        # Calculate prop and leg distances
        D_prop_values = D_perp_values - env.Forward_Reach*np.sin(theta_values)
        D_Leg_values = D_perp_values - env.L_eff*np.sin(theta_values-theta_Leg)

        # Return the negative difference to use minimize_scalar for maximization
        return np.abs(first_min(D_prop_values, theta_values) - first_min(D_Leg_values, theta_values))

    # Use minimize_scalar to find the initial velocity that maximizes the first height difference
    result = minimize_scalar(objective, bounds=(0.5,5.0), method='bounded')

    # Extract the optimal initial velocity
    optimal_V_perp_0 = result.x
    V_perp = optimal_V_perp_0


    # Solve the system with the optimal initial velocity
    y0 = [D_perp_0, -V_perp, theta_0, dtheta0]
    sol = solve_ivp(env.touchdown_ODE_MotorFilter_Old, [0, t_max], y0, t_eval=np.linspace(0, t_max, 1000))

    # Extract the results
    time = sol.t
    D_perp_values = sol.y[0]
    dD_perp_values = sol.y[1]
    theta_values = sol.y[2]
    dtheta_values = sol.y[3]

    # Apply the theta constraint
    valid_indices = theta_values < np.pi
    time = time[valid_indices]
    D_perp_values = D_perp_values[valid_indices]
    dD_perp_values = dD_perp_values[valid_indices]
    theta_values = theta_values[valid_indices]
    dtheta_values = dtheta_values[valid_indices]

    # Calculate prop and leg distances
    D_prop_values = D_perp_values - env.Forward_Reach*np.sin(theta_values)
    D_Leg_values = D_perp_values - env.L_eff*np.sin(theta_values-theta_Leg)


    # Print the optimal initial velocity and the corresponding first maxima
    print(f"Optimal Initial Velocity: {optimal_V_perp_0:.4f} m/s")
    print(f"First Prop Min Distance: {first_min(D_prop_values, theta_values):.2f} m")
    print(f"First Leg Min Distance: {first_min(D_Leg_values, theta_values):.2f} m")



    # Plotting using the axes method
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ax1.plot(time, D_perp_values, label='D_perp')
    ax1.plot(time, D_prop_values, label='D_prop')
    ax1.plot(time, D_Leg_values, label='D_Leg')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('D_perp [m]')
    # ax1.set_ylim([-0.5, 0.1])
    ax1.set_xlim([0, time[-1]])
    ax1.legend()
    ax1.set_title('Position over Time')

    ax2.plot(time, np.degrees(theta_values), label='Phi_B_O')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle [deg]')
    ax2.set_ylim([0, 180])
    ax2.set_xlim([0, time[-1]])
    ax2.legend()
    ax2.set_title('Angle over Time')

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