import numpy as np
import os

os.system("clear")


## SYSTEM CONSTANTS
g = 9.81 # [m/s^2]


# =============================
#        MASS ANALYSIS
# =============================


## BODY MASSES
CF_base = 6.8           # [g]
Motor_Assembly = 3.5    # [g]
Battery = 9.4           # [g]

m_prop = 0.31           # [g]
l_prop = 47e-3          # [m]


## LEG MASSES
m_Leg = 0.44    # [g]
l_Leg = 50e-3   # [m] 
r_leg = 2.5e-3  # [m]
 
## VICON MASSES
Vicon_Tray = 1.72           # [g]
Vicon_marker_small = 0.125  # [g]
Vicon_marker_large = 0.25   # [g]


CF_body = (CF_base + Motor_Assembly*4 + Battery + Vicon_Tray + Vicon_marker_small*3) # [g]
CF_extra = m_Leg*4              # [g]
CF_total = CF_body + CF_extra   # [g]

print(f"CF_body Mass: {CF_body:.5f} [g]")
print(f"CF_total Mass: {CF_total:.5f} [g]")

# =============================
#          PROP INERTIA
# =============================

I_xx = 1/12*m_prop*1e-3*l_Leg**2
I_yy = I_xx
I_zz = 1/2*m_Leg*1e-3*r_leg**2

print(f"I_xx/I_yy: {I_xx:.2e} [kg*m^2]")
print(f"I_zz: {I_zz:.2e} [kg*m^2]")

# =============================
#          LEG INERTIA
# =============================

I_xx = 1/12*m_Leg*1e-3*l_Leg**2
I_yy = I_xx
I_zz = 1/2*m_Leg*1e-3*r_leg**2

print(f"I_xx/I_yy: {I_xx:.2e} [kg*m^2]")
print(f"I_zz: {I_zz:.2e} [kg*m^2]")


# =============================
#        THRUST ANALYSIS
# =============================

Max_Thrust_g = 16.5*0.80*4               # [g]
Max_Thrust_N = Max_Thrust_g*1e-3*9.81    # [N]

az_max = (Max_Thrust_N - CF_total*1e-3*9.81)/(CF_total*1e-3)
print(f"Max a_z: {az_max:.2f} [m/s^2]")


# =============================
#        VEL ANALYSIS
# =============================

# vf^2 = vi^2 + 2*a*dx
dx = 1.75-0.3
vz_max = np.sqrt(0.0**2 + 2*az_max*dx)
print(f"Max vz: {vz_max:.2f} [m/s]")
