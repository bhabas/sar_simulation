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
    Phi_B_O_0 = np.radians(0)   # Initial angle in radians
    dPhi_B_O_0 = 0  # Initial angular velocity
    Z_np_0 = 0    # Initial position
    V_perp = 3.48      # Initial velocity

    env = SAR_2D_Env(Ang_Acc_range=[-90,0])

    Phi_Leg = np.radians(90-env.Gamma_eff)
    env.Plane_Angle_deg = Plane_Angle
    env.Plane_Angle_rad = np.radians(Plane_Angle)


    def first_min(values, theta_values):
        for i in range(1, len(values) - 1):

            if theta_values[i] <= -np.pi:
                break

            if values[i] < values[i - 1] and values[i] < values[i + 1]:
                return values[i]
            
        return values[0]  # Default in case no peak is found
    
    t_max = 0.9  # Maximum time in seconds

    def objective(V_perp):

        # Solve the system with the optimal initial velocity
        y0 = [Z_np_0, V_perp, Phi_B_O_0, dPhi_B_O_0]
        sol = solve_ivp(env.touchdown_ODE_MotorFilter, [0, t_max], y0, t_eval=np.linspace(0, t_max, 1000))

        # Extract the results
        time = sol.t
        Z_np_values = sol.y[0]
        dZ_np_values = sol.y[1]
        Phi_B_O_values = sol.y[2]
        dPhi_B_O_values = sol.y[3]

        # Calculate prop and leg distances
        D_perp_values = -Z_np_values
        D_prop_values = D_perp_values + env.Forward_Reach*np.sin(Phi_B_O_values - env.Plane_Angle_rad)
        D_Leg_values = D_perp_values + env.L_eff*np.sin(Phi_B_O_values - env.Plane_Angle_rad + Phi_Leg)

        # Return the negative difference to use minimize_scalar for maximization
        return np.abs(first_min(D_prop_values, Phi_B_O_values) - first_min(D_Leg_values, Phi_B_O_values))

    # Use minimize_scalar to find the initial velocity that maximizes the first height difference
    result = minimize_scalar(objective, bounds=(0.5,5.0), method='bounded')

    # Extract the optimal initial velocity
    optimal_V_perp_0 = result.x
    V_perp = optimal_V_perp_0

    # Solve the system with the optimal initial velocity
    y0 = [Z_np_0, V_perp, Phi_B_O_0, dPhi_B_O_0]
    sol = solve_ivp(env.touchdown_ODE_MotorFilter, [0, t_max], y0, t_eval=np.linspace(0, t_max, 1000))

    # Extract the results
    time = sol.t
    Z_np_values = sol.y[0]
    dZ_np_values = sol.y[1]
    Phi_B_O_values = sol.y[2]
    dPhi_B_O_values = sol.y[3]

    # Apply the theta constraint
    valid_indices = Phi_B_O_values > -np.pi
    time = time[valid_indices]
    Z_np_values = Z_np_values[valid_indices]
    dZ_np_values = dZ_np_values[valid_indices]
    Phi_B_O_values = Phi_B_O_values[valid_indices]
    dPhi_B_O_values = dPhi_B_O_values[valid_indices]

    # Calculate prop and leg distances
    D_perp_values = -Z_np_values
    D_prop_values = D_perp_values + env.Forward_Reach*np.sin(Phi_B_O_values - env.Plane_Angle_rad)
    D_Leg_values = D_perp_values + env.L_eff*np.sin(Phi_B_O_values - env.Plane_Angle_rad + Phi_Leg)


    # Print the optimal initial velocity and the corresponding first maxima
    print(f"Optimal Initial Velocity: {optimal_V_perp_0:.2f} m/s")
    print(f"First Prop Min Distance: {first_min(D_prop_values, Phi_B_O_values):.2f} m")
    print(f"First Leg Min Distance: {first_min(D_Leg_values, Phi_B_O_values):.2f} m")



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
    ax1.set_title('Distance over Time')

    ax2.plot(time, np.degrees(Phi_B_O_values), label='Phi_B_O')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Angle [deg]')
    ax2.set_ylim([0, -180])
    ax2.set_xlim([0, time[-1]])
    ax2.legend()
    ax2.set_title('Angle over Time')

    plt.tight_layout()
    plt.show(block=True)
