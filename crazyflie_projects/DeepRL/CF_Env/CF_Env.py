import numpy as np
from scipy import integrate
from scipy.optimize import fsolve
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import torch as T
import torch.nn as nn
import torch.optim as optim

import os

class CF_Env(): # Model class for Single Degree of Freedom Crazyflie

    def __init__(self):
        
        super().__init__()
        ## SET DIMENSIONAL CONSTRAINTS 
        G = 9.81        # Gravity [m/s^2]
        PD = 0.115/2    # Prop Distance from COM [m]
        I_G = 16.5717e-6    # Moment of Intertia [kg*m^2]
        self.L = 0.1
        self.gamma = np.deg2rad(30)
        M_G = 0.035 
        M_L = 0.002
        e = 0.018
        self.params = (self.L,e,self.gamma,M_G,M_L,G,PD,I_G)

        self.impact_leg = None
        self.h_ceiling = 2.1

        # INTIIALIZE MODEL POSE
        self.model_pose = self.get_pose(model_state=(0,0,0))

    def get_pose(self,model_state=[0,0,0]):           
        """Returns position data of all model lines for a given state

        Args:
            model_state (list): [x,z,theta]

        Returns:
            model_pose (tuple): Returns a tuple of all line endpoint coordinates
        """        

        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params
        x_pos,z_pos,theta = model_state


        ## DEFINE LOCATION OF MODEL ORIGIN
        CG = np.array([x_pos,z_pos])


        ## UPDATE LINE COORDINATES ( These are in body (bx^,bz^) coordinates from MO)
        P1 = np.array([ e,0]) # Hinge_1 Location
        P2 = np.array([-e,0])
        
        L1 = np.array([e + L*np.sin(gamma), # Leg_1 location
                          -L*np.cos(gamma)])
        L2 = np.array([-e - L*np.sin(gamma),
                           -L*np.cos(gamma)])

        Prop1 = np.array([ PD, 0]) # Prop_1 Location
        Prop2 = np.array([-PD, 0])

            
        ## CREATE ROTATION MATRIX FROM THETA VALUE
        R = np.array([[np.cos(-theta),-np.sin(-theta)],
                        [np.sin(-theta), np.cos(-theta)]])

        ## TRANSLATE AND ROTATE BODY COORDS INTO GLOBAL (bx^ -> ex^)
        P1 = CG + R.dot(P1)
        P2 = CG + R.dot(P2)
        L1 = CG + R.dot(L1)
        L2 = CG + R.dot(L2)
        Prop1 = CG + R.dot(Prop1)
        Prop2 = CG + R.dot(Prop2)


        ## DEFINE X AND Y COORDS
        x = np.array([CG[0],
                        P1[0],
                        P2[0],
                        L1[0], 
                        L2[0],
                        Prop1[0],
                        Prop2[0]])

        z = np.array([CG[1],
                        P1[1],
                        P2[1],
                        L1[1],
                        L2[1],
                        Prop1[1],
                        Prop2[1]])

        return (x,z)

    def generateImage(self,model_state=[0,0,0]):
        """Generates image of model for a given (x_pos,z_pos,theta) w/ (Optional L)

        Args:
            L (float, optional): Optional argument for specific leg length. Defaults to class value.
        """

        ## INIT PLOT AND MODEL LINES
        fig = plt.figure()
        ax = fig.add_subplot(111,aspect='equal',autoscale_on=False,xlim=(-1.0,1.0),ylim=(0.8,2.1))
        ax.grid()
        ceil_line = ax.axhline(2.0,color='black',linestyle='--')

        ## INIT MODEL LINES
        Line_L1, = ax.plot([],[],'k-',lw=2)     # Leg_1
        Line_L2, = ax.plot([],[],'k-',lw=2)
        Line_P1, = ax.plot([],[],'k-',lw=2)     # Hinge_1
        Line_P2, = ax.plot([],[],'k-',lw=2)
        Line_Prop1, = ax.plot([],[],'k-',lw=2)  # Prop_1
        Line_Prop2, = ax.plot([],[],'k-',lw=2)
        Dot_CG, = ax.plot([],[],'r',marker='o',markersize=3) # Model Origin Dot

        ## DRAW MODEL LINES WITH 
        x,y = self.get_pose(model_state)
    
        Line_P1.set_data([x[0],x[1]],[y[0],y[1]])
        Line_P2.set_data([x[0],x[2]],[y[0],y[2]])
        Line_L1.set_data([x[1],x[3]],[y[1],y[3]])
        Line_L2.set_data([x[2],x[4]],[y[2],y[4]])
        Line_Prop1.set_data([x[0],x[5]],[y[0],y[5]])
        Line_Prop2.set_data([x[0],x[6]],[y[0],y[6]])
        Dot_CG.set_data([x[0],y[0]])

        plt.show()

    def impact_Conversion(self,impact_state=[0,0,0,0,0,0],impact_leg=1,gamma=None,L=None):
        """Converts impact conditions to rotational initial conditions

        Args:
            impact_cond (list, optional): Impact conditions [x,Vx,z,Vz,theta,dtheta]. Defaults to [0,0,0,0].
            gamma (float, optional): Leg angle. Defaults to None.
            L (float, optional): Leg angle. Defaults to None.

        Returns:
            beta,dbeta: Rotational initial conditions
        """        

        if np.any(L) == None: # If arg not given, refer to class parameter
            L = self.L

        if np.any(gamma) == None: 
            gamma = self.gamma

        (_,e,_,M_G,M_L,G,PD,I_G) = self.params

        l = L/2
        I_c = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G

        x,vx,z,vz,theta,dtheta = impact_state

        ## SWAP THETA SIGNS TO PRESERVE MATH WORK (SDOF_Analytical_Model.pdf)
        # Convert from G_y^ to e_y^ coordinate system
        theta = -theta
        dtheta = -dtheta




        if impact_leg == 1:

            beta_0 = theta - (np.pi/2 - gamma)

            H_dtheta = I_G*dtheta                                       # Angular Momentum from dtheta
            H_vx = -M_G*vx*(2*l*np.cos(gamma+theta)-e*np.sin(theta))    # Angular momentum from vel_x
            H_vy =  M_G*vz*(2*l*np.sin(gamma+theta)+e*np.cos(theta))    # Angular momentum from vel_y

        elif impact_leg == 2:

            beta_0 = theta - (np.pi/2 + gamma)

            H_dtheta = I_G*dtheta                                       # Angular Momentum from dtheta
            H_vx = -M_G*vx*(2*l*np.cos(gamma-theta)+e*np.sin(theta))    # Angular momentum from vel_x
            H_vy = M_G*vz*(2*l*np.sin(gamma-theta)+e*np.cos(theta))     # Angular momentum from vel_y


        dbeta_0 = 1/I_c*(H_dtheta + H_vx + H_vy)


        return beta_0,dbeta_0

    def beta_prop(self,gamma=None,L=None,impact_leg=1):
        """Returns minimum beta angle for when propellar contact occurs

        Args:
            gamma (float, optional): Optional argument for specific leg angle. Defaults to class value.
            L (float, optional): Optional argument for specific leg length. Defaults to class value.

        Returns:
            beta_prop (float): Minimum valid beta angle at which propellars hit ceiling
        """        

        (_,e,_,M_G,M_L,G,PD,I_G) = self.params

        if np.any(L) == None: # If arg not given, refer to class parameter
            L = self.L

        if np.any(gamma) == None: # If arg not given, refer to class parameter
            gamma = self.gamma

        a = np.sqrt((PD-e)**2+L**2-2*(PD-e)*L*np.cos(np.pi/2-gamma))
        beta_prop = np.arccos((a**2+L**2-(PD-e)**2)/(2*a*L))

        if impact_leg == 1:
            return beta_prop

        elif impact_leg == 2:
            return np.pi - beta_prop

    def beta_landing(self,gamma=None,L=None,impact_leg=1):
        """Returns the max beta value for a given gamma and model parameters

        Args:
            gamma (float, optional): Optional argument for specific leg angle. Defaults to class value.
            L (float, optional): Optional argument for specific leg length. Defaults to class value.

        Returns:
            beta_contact: Max beta angle
        """        

        if np.any(L) == None: # If arg not given, refer to class parameter
            L = self.L

        if np.any(gamma) == None: 
            gamma = self.gamma

        (_,e,_,M_G,M_L,G,PD,I_G) = self.params

        beta_contact = np.pi/2 + gamma

        if impact_leg == 1:
            return beta_contact

        elif impact_leg == 2:
            return np.pi - beta_contact

    def PotentialEnergy_Swing(self,beta,gamma=None,L=None,impact_leg=1):
        """Returns the potential energy for a given configuration where equillibrium is V_eff = 0. Everything here
        is set in swing coordinates only, global potential energy is ignored.

        Args:
            beta (float): Angle between contact leg and the ceiling
            gamma (float, optional): Optional argument for specific leg angle. Defaults to class value.
            L (float, optional): Optional argument for specific leg length. Defaults to class value.

        Returns:
            V_eff (float): Total effective potential energy of the system for a given configuration
        """    
            
        (_,e,_,M_G,M_L,g,PD,I_G) = self.params
        

        if np.any(L) == None: # If arg not given, refer to class parameter
            L = self.L

        if np.any(gamma) == None: 
            gamma = self.gamma

        l=L/2

        ## POTENTIAL ENERGY FUNCTION
        def V_G(beta): 

            if impact_leg == 1:
                return -M_G*g*(2*l*np.sin(beta) + e*np.cos(beta-gamma))

            elif impact_leg == 2:
                return -M_G*g*(2*l*np.sin(beta) - e*np.cos(beta+gamma))


        ## FIND V_G_min FOR EACH IMPACT LEG
        beta_equil = np.arctan2((2*l + e*np.sin(gamma)),e*np.cos(gamma)) # Beta angle for when V_g is at absolute min

        if impact_leg == 1:
            beta_equil = beta_equil
            

        elif impact_leg == 2:
            beta_equil = np.pi - beta_equil

        V_G_min = V_G(beta_equil)

        ## NORMALIZE V_EFF SO EQUILLIBRIUM POINT IS V_EFF = 0
        V_eff = V_G(beta) - V_G_min

        return V_eff

    def KineticEnergy(self,dbeta,gamma=None,L=None):
        """Returns the kinetic energy of the system for a given configuration. This is only in swing coordinates.

        Args:
            dbeta (float): Angular rotation rate of beta [rad/s]
            gamma (float, optional): Leg angle. Defaults to None.
            L (float, optional): Leg length. Defaults to None.

        Returns:
            T_eff (float): Kinetic energy for a given configuration
        """        

        (_,e,_,M_G,M_L,g,PD,I_G) = self.params
        

        if np.any(L) == None: # If arg not given, refer to class parameter
            L = self.L

        if np.any(gamma) == None: 
            gamma = self.gamma

        l=L/2

        I_C = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G   # Inertia about contact point

        T_eff = 1/2*I_C*dbeta**2

        return T_eff

    def leg2body(self,beta,gamma,impact_leg=1):
        """Converts beta about pivot point to theta body

        Args:
            beta (float): Angle between leg and ceiling [rad]
            gamma (float): Angle between leg and vertical body axis [rad]

        Returns:
            theta_body (float): Body rotation angle relative to global axes
        """        
        if impact_leg == 1:
            theta_body = (np.pi/2 - gamma) + beta
            
        elif impact_leg == 2:
            theta_body = (np.pi/2 + gamma) + beta
        
        return theta_body

    def body2leg(self,theta_body,gamma,impact_leg=1):
        """Converts theta body to beta about pivot point

        Args:
            theta_body (float): Body rotation angle relative to global axes
            gamma (float): Angle between leg and vertical body axis [rad]

        Returns:
            beta (float): Angle between leg and ceiling [rad]
        """        

        if impact_leg == 1:
            beta = theta_body - (np.pi/2 - gamma)
        
        elif impact_leg == 2:
            beta = 3*np.pi/2 - (theta_body + gamma)
        
        return beta

    
    def ODE_FlightTraj(self,y,agent,events,t_span,dt):
        """Function that represents the system of 1st order ODEs for the system during
        constant velocity flight

        Args:
            t ([type]): Time array
            y ([type]): Solution array
            params (array): System parameters

        Returns:
            [dX_1,dX_2,dZ_1,dZ_2,dtheta_1,dtheta_2]: System of 1st order ODE equations
        """     

        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params
        x,vx,z,vz,theta,dtheta = y

        t = t_span[0]

        t_list = [t]
        y_list = [[x,vx,z,vz,theta,dtheta]]
        agent_list = []

        event_flags = np.zeros_like(events)

        while t<=t_span[1]:
            az = 0
            z += vz*dt
            vz += az*dt

            ax = 0
            x += vx*dt
            vx += ax*dt

            theta = theta
            dtheta = dtheta

            t += dt

            y = [x,vx,z,vz,theta,dtheta]
            
            for ii,event in enumerate(events):
                if event(t,y):
                    event_flags[ii] = 0
                else:
                    event_flags[ii] = 1

            if any(event_flags) == 1:
                break

            d_ceil = (self.h_ceiling - z)
            tau = d_ceil/vz
            OFy = -vx/d_ceil
            state = [tau,OFy,d_ceil]

            action,log_prob,val = agent.choose_action(state)

            t_list.append(t)
            y_list.append([x,vx,z,vz,theta,dtheta])
            agent_list.append([action,log_prob,val])

            if tau < agent.tau_c:
                break


        t_list = np.array(t_list)
        y_list = np.array(y_list).T
        agent_list = np.array(agent_list).T
        return t_list,y_list,agent_list,event_flags

    def ODE_flip(self,t,y,My):
        """Function that represents the system of 1st order ODEs for the system during applied Moment

        Args:
            t ([type]): Time array
            y ([type]): Solution array
            params (array): System parameters

        Returns:
            [dX_1,dX_2,dZ_1,dZ_2,dtheta_1,dtheta_2]: System of 1st order ODE equations
        """     

        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params


        X_1,X_2,Z_1,Z_2,theta_1,theta_2 = y

        if My<=0:

            dX_1 = X_2
            dX_2 = -My/(M_G*PD)*np.cos(np.pi/2-theta_1)

            dZ_1 = Z_2
            dZ_2 = -My/(M_G*PD)*np.sin(np.pi/2-theta_1)-9.81

            dtheta_1 = theta_2
            dtheta_2 = My/I_G

        else:

            dX_1 = X_2
            dX_2 = My/(M_G*PD)*np.cos(np.pi/2-theta_1)

            dZ_1 = Z_2
            dZ_2 = My/(M_G*PD)*np.sin(np.pi/2-theta_1)-9.81

            dtheta_1 = theta_2
            dtheta_2 = My/I_G

        return dX_1,dX_2,dZ_1,dZ_2,dtheta_1,dtheta_2

    def ODE_proj(self,t,y):
        """Function that represents the system of 1st order ODEs for the system

        Args:
            t ([type]): Time array
            y ([type]): Solution array
            params (array): System parameters

        Returns:
            [dX_1,dX_2,dZ_1,dZ_2,dtheta_1,dtheta_2]: System of 1st order ODE equations
        """     

        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params


        X_1,X_2,Z_1,Z_2,theta_1,theta_2 = y

        dX_1 = X_2
        dX_2 = 0

        dZ_1 = Z_2
        dZ_2 = -9.81

        dtheta_1 = theta_2
        dtheta_2 = 0

        return dX_1,dX_2,dZ_1,dZ_2,dtheta_1,dtheta_2

    def ODE_swing(self,t,y):   
        """Function that represents the system of 1st order ODEs for the system

        Args:
            t ([type]): Time array
            y ([type]): Solution array
            params (array): System parameters

        Returns:
            [dB1,dB2]: System of 1st order ODE equations
        """             
        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params
    
        l = L/2             # Half leg length [m]
        I_G = 1.65717e-5    # Moment of Intertia [kg*m^2]
        I_C = M_G*(4*l**2 + e**2 + 4*l*e*np.sin(gamma)) + I_G   # Inertia about contact point


        B1,B2 = y # Beta1,Beta2

        if self.impact_leg == 1:
            dB1 = B2
            dB2 = M_G*G/I_C*(2*l*np.cos(B1) - e*np.sin(B1-gamma))

        if self.impact_leg == 2:
            dB1 = B2
            dB2 = M_G*G/I_C*(2*l*np.cos(B1) + e*np.sin(B1+gamma))

        return dB1,dB2
    
    def calcReward(self,d_min,pad_contacts,body_contact,impact_angle):
        
        R_1 = np.clip(1/d_min,0,10)

        # if -180 <= impact_angle <= -90:
        #     R_2 = 1.0
        # elif -90 < impact_angle <= 0:
        #     R_2 = -1/90*impact_angle
        # elif 0 < impact_angle <= 90:
        #     R_2 = 1/90*impact_angle
        # elif 90 < impact_angle <= 180:
        #     R_2 = 1.0
        # else:
        #     R_2 = 0

        # ## CALC R_4 FROM NUMBER OF LEGS CONNECT

        # if pad_contacts >= 3: 
        #     if body_contact == False:
        #         R_4 = 150
        #     else:
        #         R_4 = 100
            
        # elif pad_contacts == 2: 
        #     if body_contact == False:
        #         R_4 = 50
        #     else:
        #         R_4 = 25
                
        # elif pad_contacts == 1:
        #     R_4 = 10
        
        # else:
        #     R_4 = 0.0


        # R_total = R_1*10 + R_2*10 + R_4 + 0.001

        R_total = R_1*10
        return R_total


    def solveODE(self,agent,IC=[0,0,0,0,0,0],t_span=(0,4)):
        """
        Solves a sequence of ODEs for the flip stage, projectile motion stage, and contact stage of the landing sequence

        Args:
            My (float): Flip moment in [N*m] w/ (+) being clockwise & along by^ axis
            IC (list, optional): Initial conditions. Defaults to [0,0].
            t_span (list, optional): Time span. Defaults to [0,1].

        Returns:
            sol_t (array): Returns final solution array For All Time, Always.
            sol_y (array): Returns final solution array for all system states [X,Vx,Z,Vz,theta,dtheta]
        """        

        (L,e,gamma,M_G,M_L,G,PD,I_G) = self.params
        impact_flag = False
        impact_angle = 0.0

        actions = []
        log_probs = []
        vals = []

        ## DEFINE TERMINAL EVENTS FOR ODE SOLVER
        def motor_cutoff(t,y,*argv):        # *argv collects all arguments pass since we aren't using them 
            # theta = y[4]                    # this way same func can be used w/ or w/o My terms
            x,vx,z,vz,theta,dtheta = y
            return np.deg2rad(90)-np.abs(theta)    # When angle greater than 90 deg activate cutoff
        motor_cutoff.terminal = True

        def ceiling_impact_body(t,y,*argv):

            x,vx,z,vz,theta,dtheta = y
            x_pos_list,z_pos_list = self.get_pose(model_state=[x,z,theta])
            MO_z,_,_,_,_,Prop1_z,Prop2_z = z_pos_list

            if any(x >= self.h_ceiling for x in [MO_z,Prop1_z,Prop2_z]):
                return 0
            else: 
                return 1
        ceiling_impact_body.terminal = True

        def ceiling_impact_leg1(t,y,*argv):

            x,vx,z,vz,theta,dtheta = y
            x_pos_list,z_pos_list = self.get_pose(model_state=[x,z,theta])
            _,_,_,Leg1_z,_,_,_ = z_pos_list

            if Leg1_z >= self.h_ceiling:
                return 0
            else: 
                return 1
        ceiling_impact_leg1.terminal = True

        def ceiling_impact_leg2(t,y,*argv):
            
            x,vx,z,vz,theta,dtheta = y
            x_pos_list,z_pos_list = self.get_pose(model_state=[x,z,theta])
            _,_,_,_,Leg2_z,_,_ = z_pos_list

            if Leg2_z >= self.h_ceiling:
                return 0
            else: 
                return 1
        ceiling_impact_leg2.terminal = True

        def landing_contact(t,y,*argv):
            beta=y[0]
            beta_landing = self.beta_landing(impact_leg=self.impact_leg)
            return beta-beta_landing # Trigger when this equals zero [Placeholder]
        landing_contact.terminal = True

        def prop_contact(t,y,*argv):
            beta=y[0]
            beta_prop = self.beta_prop(impact_leg=self.impact_leg)
            return beta-beta_prop
        prop_contact.terminal = True

        def falling_drone(t,y,*argv):
            x,vx,z,vz,theta,dtheta = y

            if vz <= -0.5 and z <= 1.5:
                return 0
            else:
                return 1
        falling_drone.terminal = True

        def policy_output(t,y,):
            x,vx,z,vz,theta,dtheta = y

            d_ceil = (self.h_ceiling-z)
            tau = d_ceil/vz
            OFy = -vx/d_ceil

            observation = [tau,OFy,d_ceil]

            ## PASS STATE TO POLICY NETWORK
            action,log_prob,val = agent.choose_action(observation)

            actions.append(action)
            log_probs.append(log_prob)
            vals.append(val)

            return 1
        policy_output.terminal = True

        def impact_conditions(events):

            if np.size(events[0]) > 0:   # IF BODY CONTACT
                impact_leg = 0

            elif np.size(events[1]) > 0: # ELIF LEG_1 CONTACT
                impact_leg = 1

            elif np.size(events[2]) > 0: # ELIF LEG_2 CONTACT
                impact_leg = 2

            return impact_leg

        
        


        ##############################
        #     CONSTANT VEL TRAJ.
        ##############################
        # # SOLVE ODE FOR CONSTANT VELOCITY FLIGHT UNTIL TERMINAL STATE OR TAU_TRIGGER
        sol_t,sol_y,agent_list,event_flags = self.ODE_FlightTraj(
            IC,
            agent,
            events=(ceiling_impact_body,ceiling_impact_leg1,ceiling_impact_leg2),
            t_span=[t_span[0],t_span[1]],
            dt=0.001,
        )

        # SAVE ENDING STATE AND TIME 
        state_cutoff = sol_y[:,-1]
        t_cutoff = sol_t[-1]

        ## CHECK FOR EPISODE TERMINAL EVENTS
        if any(event_flags) == 1:
            impact_flag = True
            self.impact_leg = impact_conditions(event_flags)



        # ##############################
        # #     EXECUTE BODY MOMENT
        # ##############################

        ## CHECK FOR EPISODE TERMINAL EVENTS
        if impact_flag == False:

            ## SOLVE ODE FOR FLIP MANUEVER STATE UNTIL CONTACT OR MOTOR CUTOFF
            sol_Flip = integrate.solve_ivp(
                self.ODE_flip,
                y0=state_cutoff,
                args=(agent.My,),
                events=(motor_cutoff,
                    ceiling_impact_body,ceiling_impact_leg1,ceiling_impact_leg2),
                t_span=[t_cutoff,t_span[1]],
                max_step=0.001
            )
        
            ## EXTEND SOLUTION ARRAYS
            sol_t = np.concatenate((sol_t,sol_Flip.t))
            sol_y = np.concatenate((sol_y,sol_Flip.y),axis=1)

            ## SAVE ENDING STATE AND TIME 
            state_cutoff = sol_Flip.y[:,-1]
            t_cutoff = sol_Flip.t[-1]

            ## CHECK FOR EPISODE TERMINAL EVENTS
            if np.asarray(sol_Flip.t_events[2:],dtype=object).size > 0:
                impact_flag = True
                self.impact_leg = impact_conditions(sol_Flip.t_events[1:])

        # ######################################
        # #   PROJECTILE MOTION AND ROTATION
        # ######################################

        if impact_flag == False:

            ## SOLVE ODE FOR PROJECTILE MOTION UNTIL CONTACT OR MOTOR CUTOFF
            sol_proj = integrate.solve_ivp(
                self.ODE_proj,
                y0=state_cutoff,
                args=(),
                events=(ceiling_impact_body,ceiling_impact_leg1,ceiling_impact_leg2),
                t_span=[t_cutoff,t_span[1]],
                max_step=0.001
            )

            ## EXTEND SOLUTION ARRAYS
            sol_t = np.concatenate((sol_t,sol_proj.t))
            sol_y = np.concatenate((sol_y,sol_proj.y),axis=1)


            ## SAVE ENDING STATE AND TIME
            state_cutoff = sol_proj.y[:,-1]
            t_cutoff = sol_proj.t[-1]

            ## CHECK FOR EPISODE TERMINAL EVENTS
            if np.asarray(sol_proj.t_events[:],dtype=object).size > 0:
                impact_flag = True
                impact_angle = np.degrees(state_cutoff[4]) # [deg]
                self.impact_leg = impact_conditions(sol_proj.t_events[:])

        ########################
        #   IMPACT DYNAMICS
        ########################

        if self.impact_leg == 0: ## BODY CONTACT
            # print("Failed Landing (Body Contact)")
            pad_contacts = 0
            body_contact = True
    
        elif self.impact_leg == 1:

            beta_0,dbeta_0 = self.impact_Conversion(state_cutoff,self.impact_leg)

            ## FIND IMPACT COORDS
            r_G_C1_0 = np.array(
                        [[-(L+e*np.sin(gamma))*np.cos(beta_0)+e*np.cos(gamma)*np.sin(beta_0)],
                        [-(L+e*np.sin(gamma))*np.sin(beta_0)-e*np.cos(gamma)*np.cos(beta_0)]])

            r_O_G = np.array([[state_cutoff[0]],[state_cutoff[2]]])
            r_O_C1 = r_O_G - r_G_C1_0
            
            ## SOLVE SWING ODE
            sol_Swing = integrate.solve_ivp(
                self.ODE_swing,
                y0=[beta_0,dbeta_0],
                args=(),
                events=(landing_contact,prop_contact),
                t_span=[t_cutoff,t_span[1]],
                max_step=0.001)

            
            beta,dbeta = sol_Swing.y

            ## SOLVE FOR SWING BEHAVIOR IN GLOBAL COORDINATES
            r_G_C1 = np.array(
                        [[-(L+e*np.sin(gamma))*np.cos(beta)+e*np.cos(gamma)*np.sin(beta)],
                        [-(L+e*np.sin(gamma))*np.sin(beta)-e*np.cos(gamma)*np.cos(beta)]])

            theta = -((np.pi/2-gamma) + beta) # Convert beta in (e^) to theta in (G^)

            sol_swing_y = np.zeros((6,len(beta)))

            sol_swing_y[0,:] = r_O_C1[0] + r_G_C1[0,:]  # x
            sol_swing_y[2,:] = r_O_C1[1] + r_G_C1[1,:]  # z
            sol_swing_y[4,:] = theta                    # theta

            ## COMBINE SOLUTION ARRAYS
            sol_t = np.concatenate((sol_t,sol_Swing.t))
            sol_y = np.concatenate((sol_y,sol_swing_y),axis=1)


            ## CHECK TERMINAL STATES
            if np.asarray(sol_Swing.t_events[0],dtype=object).size > 0:  # SUCCESSFUL LANDING CONTACT
                # print("Successful Landing")
                pad_contacts = 4
                body_contact = False

            elif np.asarray(sol_Swing.t_events[1],dtype=object).size > 0:  # BODY CONTACT
                # print("Failed Landing (Prop Contact)")
                pad_contacts = 2
                body_contact = True

            else: # ONLY IMPACT LEG CONTACT (FREE SWING)
                # print("Failed Landing (No Swing Contact)")
                pad_contacts = 2
                body_contact = False


        elif self.impact_leg == 2:

            beta_0,dbeta_0 = self.impact_Conversion(state_cutoff,self.impact_leg)

            ## FIND IMPACT POINT
            r_G_C2_0 = np.array(
                        [[-(L+e*np.sin(gamma))*np.cos(beta_0)-e*np.cos(gamma)*np.sin(beta_0)],
                        [-(L+e*np.sin(gamma))*np.sin(beta_0)+e*np.cos(gamma)*np.cos(beta_0)]])

            r_O_G = np.array([[state_cutoff[0]],[state_cutoff[2]]])
            r_O_C2 = r_O_G - r_G_C2_0
            
            ## SOLVE SWING ODE
            sol_Swing = integrate.solve_ivp(
                self.ODE_swing,
                y0=[beta_0,dbeta_0],
                args=(),
                events=(landing_contact,prop_contact),
                t_span=[t_cutoff,t_span[1]],
                max_step=0.001)

            
            beta,dbeta = sol_Swing.y


            ## SOLVE FOR SWING BEHAVIOR IN GLOBAL COORDINATES
            r_G_C2 = np.array(
                        [[-(L+e*np.sin(gamma))*np.cos(beta)-e*np.cos(gamma)*np.sin(beta)],
                        [-(L+e*np.sin(gamma))*np.sin(beta)+e*np.cos(gamma)*np.cos(beta)]])

            theta = -((np.pi/2+gamma) + beta) # Convert beta in (e^) to theta in (G^)

            sol_swing_y = np.zeros((6,len(beta)))

            sol_swing_y[0,:] = r_O_C2[0] + r_G_C2[0,:]  # x
            sol_swing_y[2,:] = r_O_C2[1] + r_G_C2[1,:]  # z
            sol_swing_y[4,:] = theta                    # theta

            ## COMBINE SOLUTION ARRAYS
            sol_t = np.concatenate((sol_t,sol_Swing.t))
            sol_y = np.concatenate((sol_y,sol_swing_y),axis=1)


            ## CHECK TERMINAL STATES
            if np.asarray(sol_Swing.t_events[0],dtype=object).size > 0:  # SUCCESSFUL LANDING CONTACT
                # print("Successful Landing")
                pad_contacts = 4
                body_contact = False

            elif np.asarray(sol_Swing.t_events[1],dtype=object).size > 0:  # BODY CONTACT
                # print("Failed Landing (Prop Contact)")
                pad_contacts = 2
                body_contact = True

            else: # ONLY IMPACT LEG CONTACT (FREE SWING)
                # print("Failed Landing (No Swing Contact)")
                pad_contacts = 2
                body_contact = False

        else:
            # print("Failed Landing (No Contact)")
            pad_contacts = 0
            body_contact = False

            
        ## CALCULATE REWARD
        x,dx,z,dz,theta,dthetea = sol_y
        d_ceil = (self.h_ceiling+0.05 - z)
        d_ceil_min = np.min(d_ceil)

        impact_angle = impact_angle

        actions,log_probs,vals = agent_list
        
        rewards = np.zeros(len(actions))
        rewards[-1] = self.calcReward(d_ceil_min,pad_contacts,body_contact,impact_angle)

        dones = np.zeros(len(actions))
        dones[-1] = 1
        

        state = [sol_t,sol_y]
        
        return state,actions,log_probs,vals,rewards,dones

    def animateTraj(self,states):
        """Create animation of model trajectory

        """        

        ## RESHAPE/INTERPOLATE DATA TO FIXED TIME STEP
        step_size = 0.005 # [s]
        sol_t,sol_y = states
        interp = interp1d(sol_t,sol_y,axis=1)
        sol_t = np.arange(sol_t[0],sol_t[-1],step_size)
        sol_y = interp(sol_t)

        ## ANIMATION FUNCTIONS
        def init_lines(): ## INITIALIZE ANIMATED LINES

            Line_CG.set_data([],[])
            Line_P1.set_data([],[])
            Line_P2.set_data([],[])
            Line_L1.set_data([],[])
            Line_L2.set_data([],[])
            Line_Prop1.set_data([],[])
            Line_Prop2.set_data([],[])
            

            return Line_P1,Line_P2,Line_L1,Line_L2,Line_Prop1,Line_Prop2,Line_CG  


        def animate_func(i,time,model_states):
            x,y = self.get_pose(model_states[:,i])
            
            Line_CG.set_data([x[0]],[y[0]])
            Line_P1.set_data([x[0],x[1]],[y[0],y[1]])
            Line_P2.set_data([x[0],x[2]],[y[0],y[2]])
            Line_L1.set_data([x[1],x[3]],[y[1],y[3]])
            Line_L2.set_data([x[2],x[4]],[y[2],y[4]])
            Line_Prop1.set_data([x[0],x[5]],[y[0],y[5]])
            Line_Prop2.set_data([x[0],x[6]],[y[0],y[6]])
            
            
            title.set_text(f"Time: {time[i]*1000:.2f} [ms]")



            return Line_P1,Line_P2,Line_L1,Line_L2,Line_Prop1,Line_Prop2,Line_CG,title  


        ## CONVERT GLOBAL STATES TO MODEL STATES
        model_states = np.array([sol_y[0],sol_y[2],sol_y[4]])
        

        ## INIT PLOT AND MODEL LINES
        fig = plt.figure()
        ax = fig.add_subplot(111,aspect='equal',autoscale_on=True)
        ax.grid()
        ax.set_xlim(-0.5,2)
        ax.set_ylim(-0.1,2.5)
        ground_plane = ax.axhline(0,color='black',linestyle='--')
        ceiling_plane = ax.axhline(self.h_ceiling,color='black',linestyle='--')

        title = ax.text(0.5,0.9,"",bbox={'facecolor':'w','pad':5},transform=ax.transAxes,ha="center")


        Line_CG, = ax.plot([],[],'r',marker='o',markersize=3)
        Line_P1, = ax.plot([],[],'k-',lw=2)
        Line_P2, = ax.plot([],[],'k-',lw=2)
        Line_L1, = ax.plot([],[],'k-',lw=2)
        Line_L2, = ax.plot([],[],'k-',lw=2)
        Line_Prop1, = ax.plot([],[],'k-',lw=2)
        Line_Prop2, = ax.plot([],[],'k-',lw=2)

        ani = animation.FuncAnimation(fig,animate_func,fargs=(sol_t,model_states),
                                    init_func=init_lines,
                                    frames=len(sol_t),
                                    interval=10,
                                    blit=True,
                                    repeat=False)

        plt.show(block=False)
        plt.pause(10e-3*len(sol_t))
        plt.close()
