import matplotlib.pyplot as plt
import numpy as np


Plane_Angle = 0

gamma = 45
L = 150e-3
PD = 75e-3

a = np.sqrt(PD**2 + L**2 - 2*PD*L*np.cos(90-gamma))
Beta_min = np.arccos((L**2 + a**2 - PD**2)/(2*a*L))



## POLICY TRIGGER PLOT
def Reward_Exp_Decay(x,k,threshold):
    if 0 < x < threshold:
        return 1
    elif threshold <= x:
        return np.exp(-k*(x-threshold))
    

## REWARD: TAU_CR TRIGGER
x = np.linspace(0,2,500)
R = np.vectorize(Reward_Exp_Decay)(x,5,0.5)
    
fig = plt.figure()
ax1 = fig.add_subplot()
ax1.plot(x,R)
ax1.set_title("Reward: Tau_CR_Trg")
ax1.set_xlabel("Tau_CR_Trg")
ax1.set_ylabel("Reward")
ax1.set_xlim(-0.5,3)
ax1.set_ylim(-0.5,1.5)
ax1.hlines(0,-300,300)
ax1.vlines(0,-5,5)
ax1.grid()
plt.show()


## REWARD: MINIMUM DISTANCE 
x = np.linspace(0,2,500)
R = np.vectorize(Reward_Exp_Decay)(x,5,0.25)
    
fig = plt.figure()
ax2 = fig.add_subplot()
ax2.plot(x,R)
ax2.set_title("Reward: D_perp_min")
ax2.set_xlabel("D_perp_min")
ax2.set_ylabel("Reward")
ax2.set_xlim(-0.5,3)
ax2.set_ylim(-0.5,1.5)
ax2.hlines(0,-300,300)
ax2.vlines(0,-5,5)
ax2.grid()
plt.show()

## REWARD: MOMENTUM TRANSFER
def Reward_LT(x_deg,Leg_Num):
    x_rad = np.radians(x_deg)

    if Leg_Num == 2:
        x_deg = -x_deg  # Reflect across the y-axis
        x_rad = -x_rad  # Reflect the radian value as well


    if -180 <= x_deg <= 0:
        return -np.sin(x_rad)
    elif 0 < x_deg <= 180:
        return -0.5/180 * x_deg

    return None
    
x = np.linspace(-180,180,500)
R_Leg1 = np.vectorize(Reward_LT)(x,Leg_Num=1)
R_Leg2 = np.vectorize(Reward_LT)(x,Leg_Num=2)
    
fig = plt.figure()
ax3 = fig.add_subplot()
ax3.plot(x,R_Leg1,label="Leg 1")
ax3.plot(x,R_Leg2,label="Leg 2")

ax3.set_xlim(-200,200)
ax3.set_ylim(-2,2)
ax3.set_xticks(np.arange(-180,180,45))
ax3.set_title("Reward: Angular Momentum Transfer")
ax3.set_xlabel("Angle: v x e_r")
ax3.set_ylabel("Reward")
ax3.hlines(0,-300,300)
ax3.vlines(0,-5,5)
ax3.legend()
ax3.grid()
# plt.show()


## REWARD: GRAVITY MOMENT 
def Reward_GravityMoment(x_deg,Leg_Num):
    x_rad = np.radians(x_deg)

    if Leg_Num == 2:
        x_deg = -x_deg  # Reflect across the y-axis
        x_rad = -x_rad  # Reflect the radian value as well

    return -np.sin(x_rad)

x = np.linspace(-180,180,500)
R_Leg1 = np.vectorize(Reward_GravityMoment)(x,Leg_Num=1)
R_Leg2 = np.vectorize(Reward_GravityMoment)(x,Leg_Num=2)
    
fig = plt.figure()
ax4 = fig.add_subplot()
ax4.plot(x,R_Leg1,label="Leg 1")
ax4.plot(x,R_Leg2,label="Leg 2")

ax4.set_xlim(-200,200)
ax4.set_ylim(-2,2)
ax4.set_xticks(np.arange(-180,180,45))
ax4.set_title("Reward: Gravity Moment")
ax4.set_xlabel("Angle: g x e_r")
ax4.set_ylabel("Reward")
ax4.legend()
ax4.hlines(0,-300,300)
ax4.vlines(0,-5,5)
ax4.grid()
plt.show()

## REWARD: IMPACT ANGLE
def Reward_ImpactAngle(phi,phi_min,Leg_Num=1,phi_thr=200):

    phi_max = 180
    phi_b = (phi_min + phi_max)/2

    if Leg_Num == 2:
        phi = -phi
        

    if -phi_min < phi <= phi_min:
        return 0.5*phi/phi_min
    elif phi_min < phi <= phi_b:
        return 0.5/(phi_b - phi_min) * (phi - phi_min) + 0.5
    elif phi_b < phi <= phi_max:
        return -0.5/(phi_max - phi_b) * (phi - phi_b) + 1.0
    elif phi_max < phi <= phi_thr:
        return -1/(phi_thr - phi_max) * (phi - phi_max) + 0.5
    else:
        return -0.5


x = np.linspace(-300,300,500)
phi_min = 90
R_Leg1 = np.vectorize(Reward_ImpactAngle)(x,phi_min,Leg_Num=1)
R_Leg2 = np.vectorize(Reward_ImpactAngle)(x,phi_min,Leg_Num=2)
    
fig = plt.figure()
ax5 = fig.add_subplot()
ax5.plot(x,R_Leg1,label="Leg 1")
ax5.plot(x,R_Leg2,label="Leg 2")

ax5.set_xticks(np.linspace(-225,225,11))
ax5.set_xlim(-270,270)
ax5.set_ylim(-2,2)
ax5.set_title("Reward: Impact Angle Window")
ax5.set_xlabel("Angle: Phi_rel")
ax5.set_ylabel("Reward")
ax5.hlines(0,-300,300)
ax5.vlines(0,-5,5)
ax5.legend()
ax5.grid()
plt.show()
