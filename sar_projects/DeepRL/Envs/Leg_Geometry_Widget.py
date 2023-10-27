import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button, TextBox

import matplotlib.patches as patches

import matplotlib.gridspec as gridspec

import matplotlib
# matplotlib.use('Qt5Agg')


class InteractivePlot:
    def __init__(self):

        ## INITIAL VALUES
        gamma_deg = 45
        gamma_rad = np.deg2rad(gamma_deg)         
        self.Contact_Leg = 1
        LP_Ratio = 1.4
        Plane_Angle_deg = 135

        ## CONFIGS
        self.Show_Vel_vec = False
        self.Show_grav_vec = False

        ## SAR DIMENSIONAL CONSTRAINTS
        self.PD = 75e-3             # Prop Distance from COM [m]

        self.create_ax()
        self.init_plots()
        self.Update_1(None) 

    def create_ax(self):

        self.fig = plt.figure(figsize=(14,10),constrained_layout=False)
        widths = [0.125, 0.1, 5, 0.5, 4]
        heights = [3,3,3,0.5,0.5,0.5]
        gs = self.fig.add_gridspec(ncols=len(widths), nrows=len(heights), width_ratios=widths, height_ratios=heights, hspace=1.0)
        

        ## QUAD AX
        self.ax_Quad = self.fig.add_subplot(gs[0:3,2])
        self.ax_Quad.set_xlim(-0.5,0.5)
        self.ax_Quad.set_ylim(-0.5,0.5)
        self.ax_Quad.set_aspect('equal', 'box')

        ## QUAD SLIDERS
        self.ax_Gamma_Slider = self.fig.add_subplot(gs[0:3,0])
        self.Gamma_Slider = Slider(
            ax=self.ax_Gamma_Slider,
            label="Gamma [deg]",
            valmin=0,
            valmax=90,
            valinit=30,
            valstep=1,
            orientation="vertical"
        )


        self.ax_Leg_Slider = self.fig.add_subplot(gs[3,2])
        self.Leg_Slider = Slider(
            ax=self.ax_Leg_Slider,
            label='Leg Prop \n Ratio [m]',
            valmin=0.1,
            valmax=4,
            valinit=1.5,
            valstep=0.1
        )

        ## PLANE SLIDER
        self.ax_Plane_Slider = self.fig.add_subplot(gs[3,-1])
        self.Plane_Angle_Slider = Slider(
            ax=self.ax_Plane_Slider,
            label='Plane Angle \n [deg]',
            valmin=0,
            valmax=270,
            valinit=0,
            valstep=1
        )

        ## BETA SLIDER
        self.ax_Beta_Slider = self.fig.add_subplot(gs[4,-1])
        self.Beta_Slider = Slider(
            ax=self.ax_Beta_Slider,
            label='Beta Angle\n [deg]',
            valmin=-180,
            valmax=90,
            valinit=0,
            valstep=1
        )

        ## FLIGHT ANGLE SLIDER
        self.ax_V_Angle_Slider = self.fig.add_subplot(gs[5,-1])
        self.Flight_Angle_Slider = Slider(
            ax=self.ax_V_Angle_Slider,
            label='Flight Angle \n [deg]',
            valmin=5,
            valmax=175,
            valinit=0,
            valstep=1
        )


        ## CONFIGS
        gs_buttons = gridspec.GridSpecFromSubplotSpec(3, 3, subplot_spec=gs[4:,1:3], wspace=0.5, hspace=0.5)

        ## LEG BUTTON
        ax_Leg_Button = self.fig.add_subplot(gs_buttons[0, 0])
        self.leg_button = Button(ax_Leg_Button, 'Switch Leg', hovercolor='0.975')
        self.leg_button.on_clicked(self.Switch_Leg)

        ## VELOCITY BUTTON
        ax_Vel_Button = self.fig.add_subplot(gs_buttons[1, 0])
        self.vel_button = Button(ax_Vel_Button, 'Vel. Vector', hovercolor='0.975')
        self.vel_button.on_clicked(self.Vel_Visible)

        ax_Gravity_Button = self.fig.add_subplot(gs_buttons[2, 0])
        self.gravity_button = Button(ax_Gravity_Button, 'Grav. Vector', hovercolor='0.975')
        self.gravity_button.on_clicked(self.Grav_Visible)

        ## TEXT BOXES
        ax_Impact_Window_text = self.fig.add_subplot(gs_buttons[0, 1])
        ax_Impact_Window_text.axis('off')
        self.Impact_Window_text = ax_Impact_Window_text.text(0,0.5,f"Impact Window: {0.0:3.1f} [deg]")

        ax_Phi_text = self.fig.add_subplot(gs_buttons[1, 1])
        ax_Phi_text.axis('off')
        self.Phi_text = ax_Phi_text.text(0,0.5,f"Phi: {0.0: 3.1f} [deg]")

        ax_Phi_rel_text = self.fig.add_subplot(gs_buttons[2, 1])
        ax_Phi_rel_text.axis('off')
        self.Phi_rel_text = ax_Phi_rel_text.text(0,0.5,f"Phi_rel: {0.0: 3.1f} [deg]")

        ax_Temp_GV_text = self.fig.add_subplot(gs_buttons[0, 2])
        ax_Temp_GV_text.axis('off')
        self.Temp_GV_text = ax_Temp_GV_text.text(0,0.5,f"(g x v): {0.0: 3.1f} [deg]")



        self.R_LT_ax  = self.fig.add_subplot(gs[0,-1])
        self.R_GM_ax  = self.fig.add_subplot(gs[1,-1])
        self.R_phi_ax = self.fig.add_subplot(gs[2,-1])


        ## UPDATE PLOTS
        self.Plane_Angle_Slider.on_changed(self.Update_1) 
        self.Leg_Slider.on_changed(self.Update_1)
        self.Gamma_Slider.on_changed(self.Update_1)

        self.Beta_Slider.on_changed(self.Update_2)  
        self.Flight_Angle_Slider.on_changed(self.Update_2) 

    def init_plots(self):

        ## GEOMETRIC CONSTRAINTS
        L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)

        ## QUAD LINES
        CG,L1,L2,Prop1,Prop2 = self._get_pose(0,0,0)
        CG_Marker = patches.Circle(CG,radius=0.01,fc='tab:blue',zorder=10)
        self.CG_Marker = self.ax_Quad.add_patch(CG_Marker)


        self.leg1_line,  = self.ax_Quad.plot([],[],lw=2,zorder=5,c='red')
        self.leg2_line,  = self.ax_Quad.plot([],[],lw=2,zorder=5,c='black')
        self.prop1_line, = self.ax_Quad.plot([],[],lw=2,zorder=5,c='black')
        self.prop2_line, = self.ax_Quad.plot([],[],lw=2,zorder=5,c='black')

        Swing_Arc = patches.Arc(
            xy=CG,width=L*2,
            height=L*2,
            color="tab:gray",
            zorder=2,
            linestyle="--"
        )
        self.Swing_Arc = self.ax_Quad.add_patch(Swing_Arc)

        ## PLANE LINES
        self.Plane_line, = self.ax_Quad.plot([],[],lw=1,zorder=1,c="k")
        self.n_p_line,   = self.ax_Quad.plot([],[],lw=3,zorder=2,c="tab:green")
        self.t_x_line,   = self.ax_Quad.plot([],[],lw=3,zorder=2,c="tab:blue")

        ## VELOCITY LINES
        self.Vel_line, = self.ax_Quad.plot([],[],c="tab:green", lw=2,zorder=8)
        self.Vel_traj_line, = self.ax_Quad.plot([],[],c="tab:grey",linestyle='--',lw=1,zorder=1,alpha=0.7)
        self.Grav_line, = self.ax_Quad.plot([],[],c="tab:purple", lw=2,zorder=5)

        ## MOMENTUM TRANSFER PLOT
        x = np.linspace(-180,180,500)
        R_Leg1 = np.vectorize(self.Reward_LT)(x,Leg_Num=1)
        R_Leg2 = np.vectorize(self.Reward_LT)(x,Leg_Num=2)
        self.R_LT_ax.plot(x,R_Leg1,label="Leg 1")
        self.R_LT_ax.plot(x,R_Leg2,label="Leg 2")
        self.R_LT_ax.set_xlim(-200,200)
        self.R_LT_ax.set_ylim(-0.75,1.25)
        self.R_LT_ax.set_xticks(np.arange(-180,181,45))
        self.R_LT_ax.set_yticks(np.arange(-1,1.1,0.5))
        self.R_LT_ax.set_title("Reward: Angular Momentum Transfer")
        self.R_LT_ax.set_xlabel("Angle: (v x e_r)")
        self.R_LT_ax.set_ylabel("Reward")
        self.R_LT_ax.hlines(0,-300,300)
        self.R_LT_ax.vlines(0,-5,5)
        self.R_LT_ax.legend(loc="lower right",ncol=2)
        self.R_LT_ax.grid()
        self.R_LT_dot, = self.R_LT_ax.plot([],[],'ro')


        ## GRAVITY MOMENT PLOT
        x = np.linspace(-180,180,500)
        R_Leg1 = np.vectorize(self.Reward_GravityMoment)(x,Leg_Num=1)
        R_Leg2 = np.vectorize(self.Reward_GravityMoment)(x,Leg_Num=2)
        self.R_GM_ax.plot(x,R_Leg1,label="Leg 1")
        self.R_GM_ax.plot(x,R_Leg2,label="Leg 2")
        self.R_GM_ax.set_xlim(-200,200)
        self.R_GM_ax.set_ylim(-1.25,1.25)
        self.R_GM_ax.set_xticks(np.arange(-180,181,45))
        self.R_GM_ax.set_yticks(np.arange(-1,1.1,0.5))
        self.R_GM_ax.set_title("Reward: Gravity Moment")
        self.R_GM_ax.set_xlabel("Angle: (g x e_r)")
        self.R_GM_ax.set_ylabel("Reward")
        self.R_GM_ax.legend(loc="lower right",ncol=2)
        self.R_GM_ax.hlines(0,-300,300)
        self.R_GM_ax.vlines(0,-5,5)
        self.R_GM_ax.grid()
        self.R_GM_dot, = self.R_GM_ax.plot([],[],'ro')



        


    def Update_1(self,val):

        ## READ GAMMA, LENGTH
        L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)

        ## PLANE DRAW
        Plane_Angle_deg = self.Plane_Angle_Slider.val
        Plane_Angle_rad = np.radians(Plane_Angle_deg)

        ## UPDATE PLANE DRAWING
        vec = np.array([1,0])
        vec = self.R_PW(vec,np.radians(self.Plane_Angle_Slider.val))
        self.Plane_line.set_data([-vec[0],vec[0]],[-vec[1],vec[1]])

        t_x = 0.05*np.array([1,0])
        t_x = self.R_PW(t_x,np.radians(self.Plane_Angle_Slider.val))
        self.t_x_line.set_data([0,t_x[0]],[0,t_x[1]])

        n_p = 0.05*np.array([0,1])
        n_p = self.R_PW(n_p,np.radians(self.Plane_Angle_Slider.val))
        self.n_p_line.set_data([0,n_p[0]],[0,n_p[1]])


        ## SOLVE FOR MINIMUM BETA ANGLE VIA GEOMETRIC CONSTRAINTS
        a = np.sqrt(self.PD**2 + L**2 - 2*self.PD*L*np.cos(np.pi/2-gamma_rad))
        self.Beta_min_rad = np.arccos((L**2 + a**2 - self.PD**2)/(2*a*L))

        if self.Contact_Leg == 1:
            self.Beta_min_rad = -self.Beta_min_rad # Swap sign to match coordinate notation
            self.Beta_min_deg = np.rad2deg(self.Beta_min_rad)

            self.Beta_Slider.valmax = self.Beta_min_deg
            self.Beta_Slider.valmin = -(90 + gamma_deg)

        elif self.Contact_Leg == 2:
            self.Beta_min_rad = -(np.pi - self.Beta_min_rad)
            self.Beta_min_deg = np.rad2deg(self.Beta_min_rad)

            self.Beta_Slider.valmax = -(90 - gamma_deg)
            self.Beta_Slider.valmin = self.Beta_min_deg

        self.Beta_Slider.set_val(self.Beta_min_deg)        

    def Update_2(self,val):

        ## READ GAMMA, LENGTH
        L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)

        ## UPDATE PLANE ANGLE VALUE
        Plane_Angle_deg = self.Plane_Angle_Slider.val
        Plane_Angle_rad = np.radians(Plane_Angle_deg)

        ## UPDATE BETA VALUE
        Beta_deg = self.Beta_Slider.val
        Beta_rad = np.radians(Beta_deg)

        ## UPDATE VELOCITY LINE VECTOR
        Vel_angle = np.radians(self.Flight_Angle_Slider.val)
        V_B_P = np.array([np.cos(Vel_angle),np.sin(Vel_angle)])
        V_B_O = self.R_PW(V_B_P,Plane_Angle_rad)
        V_hat = V_B_O

        ## GRAVITY VECOTR
        g_hat = np.array([0,-1]) # {X_W,Z_W}


        if self.Contact_Leg == 1:
            phi_rad = np.arctan2(-np.cos(Beta_rad + gamma_rad + Plane_Angle_rad), \
                                np.sin(Beta_rad + gamma_rad + Plane_Angle_rad))
            phi_deg = np.rad2deg(phi_rad)

            phi_rel_rad = phi_rad - Plane_Angle_rad
            phi_rel_deg = np.rad2deg(phi_rel_rad)

            ## UPDATE ARC VALUES
            self.Swing_Arc.width = self.Swing_Arc.height = L*2
            self.Swing_Arc.angle = 180-Plane_Angle_deg
            self.Swing_Arc.theta1 = np.abs(self.Beta_min_deg)
            self.Swing_Arc.theta2 = 90 + gamma_deg

            ## UPDATE BODY POSITION
            r_B_C1 = np.array([-L,0])                                       # {e_r1,e_beta1}
            r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta_rad),Plane_Angle_rad) # {X_W,Z_W}
            r_B_O = r_B_C1   
    
            r_C1_B = -r_B_C1                       # {X_W,Z_W}
            e_r1 = r_C1_B/np.linalg.norm(r_C1_B)   # {X_W,Z_W}
            e_r_hat = np.array([e_r1[0],e_r1[1]])

            ## MOMENTUM TRANSFER REWARD
            CP_LT = -np.cross(V_hat,e_r_hat) # Swap sign for consitent notation
            DP_LT = np.dot(V_hat,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))
            R_LT = self.Reward_LT(CP_LT_angle_deg,Leg_Num=1)

            ## GRAVITY MOMENT REWARD
            CP_GM = -np.cross(g_hat,e_r_hat) # Swap sign for consitent notation
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,Leg_Num=1)
           

        elif self.Contact_Leg == 2:

            phi_rad = np.arctan2(-np.cos(Beta_rad - gamma_rad + Plane_Angle_rad), \
                              np.sin(Beta_rad - gamma_rad + Plane_Angle_rad))
            phi_deg = np.rad2deg(phi_rad)

            phi_rel_rad = phi_rad - Plane_Angle_rad
            phi_rel_deg = np.rad2deg(phi_rel_rad)

            ## UPDATE ARC VALUES
            self.Swing_Arc.width = self.Swing_Arc.height = L*2
            self.Swing_Arc.angle = 180-Plane_Angle_deg
            self.Swing_Arc.theta1 = 90 - gamma_deg
            self.Swing_Arc.theta2 = np.abs(self.Beta_min_deg)

            r_B_C2 = np.array([-L,0])                                       # {e_r1,e_beta1}
            r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta_rad),Plane_Angle_rad) # {X_W,Z_W}
            r_B_O = r_B_C2

            r_C2_B = -r_B_C2                       # {X_W,Z_W}
            e_r2 = r_C2_B/np.linalg.norm(r_C2_B)   # {X_W,Z_W}
            e_r_hat = np.array([e_r2[0],e_r2[1]])

            ## MOMENTUM TRANSFER REWARD
            CP_LT = -np.cross(V_hat,e_r_hat) # Swap sign for consitent notation
            DP_LT = np.dot(V_hat,e_r_hat)
            CP_LT_angle_deg = np.degrees(np.arctan2(CP_LT,DP_LT))
            R_LT = self.Reward_LT(CP_LT_angle_deg,Leg_Num=2)

            ## GRAVITY MOMENT REWARD
            CP_GM = -np.cross(g_hat,e_r_hat) # Swap sign for consitent notation
            DP_GM = np.dot(g_hat,e_r_hat)
            CP_GM_angle_deg = np.degrees(np.arctan2(CP_GM,DP_GM))
            R_GM = self.Reward_GravityMoment(CP_GM_angle_deg,Leg_Num=2)

            
        


        ## UPDATE BODY DRAWING
        CG,L1,L2,Prop1,Prop2 = self._get_pose(r_B_O[0],r_B_O[1],phi_rad)
        self.leg1_line.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2_line.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1_line.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2_line.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])
        self.CG_Marker.set_center(CG)


        

        if self.Show_Vel_vec == True:
            self.Vel_line.set_data([CG[0],CG[0] + 0.08*V_B_O[0]],[CG[1],CG[1] + 0.08*V_B_O[1]])
            self.Vel_traj_line.set_data([CG[0] + 5*V_B_O[0],CG[0] - 10*V_B_O[0]],[CG[1] + 5*V_B_O[1],CG[1] - 10*V_B_O[1]])
        else:
            self.Vel_line.set_data([None,None],[None,None])
            self.Vel_traj_line.set_data([None,None],[None,None])

        ## UPDATE GRAVITY VECTOR
        if self.Show_grav_vec == True:
            self.Grav_line.set_data([CG[0],CG[0]],[CG[1],CG[1]-0.08])
        else:
            self.Grav_line.set_data([None,None],[None,None])


        ## UPDATE TEXT BOX
        impact_window = np.abs(self.Swing_Arc.theta2 - self.Swing_Arc.theta1)
        self.Impact_Window_text.set_text(f"Impact Window: {impact_window:3.1f} [deg]")

        self.Phi_text.set_text(f"Phi: {phi_deg: 3.1f} [deg]")
        self.Phi_rel_text.set_text(f"Phi_rel: {phi_rel_deg: 3.1f} [deg]")

        ## G X V TEMP TEXT BOX
        g_hat = np.array([0,-1])
        CP_temp = -np.cross(g_hat,V_hat)  # Swap sign for consitent notation
        DP_temp = np.dot(g_hat,V_hat)
        CP_temp_angle_deg = np.degrees(np.arctan2(CP_temp,DP_temp))
        self.Temp_GV_text.set_text(f"(g x v): {CP_temp_angle_deg: 3.1f} [deg]")

        ## MOMENTUM TRANSFER REWARD
        self.R_LT_dot.set_data(CP_LT_angle_deg,R_LT)

        ## GRAVITY MOMENT REWARD
        self.R_GM_dot.set_data(CP_GM_angle_deg,R_GM)




        self.fig.canvas.draw_idle()

    def Switch_Leg(self,event):

        if self.Contact_Leg == 1:
            self.Contact_Leg = 2
        else:
            self.Contact_Leg = 1

        self.Update_1(None)

    def Vel_Visible(self,event):

        if self.Show_Vel_vec == True:
            self.Show_Vel_vec = False
            self.R_LT_ax.set_visible(False)

        else:
            self.Show_Vel_vec = True
            self.R_LT_ax.set_visible(True)

        self.Update_2(None)

    def Grav_Visible(self,event):
        
        if self.Show_grav_vec == True:
            self.Show_grav_vec = False
            self.R_GM_ax.set_visible(False)
        else:
            self.Show_grav_vec = True
            self.R_GM_ax.set_visible(True)

        self.Update_2(None)

    def show(self):
        plt.show()

    def Reward_ImpactAngle(self,phi_rel,phi_min,rot_dir=-1,phi_thr=-200):

        phi_max = -180
        phi_b = (phi_min + phi_max)/2

        if rot_dir == +1:
            phi_rel = -phi_rel

        if phi_rel <= phi_thr:
            return -0.5
        elif phi_thr < phi_rel <= phi_max:
            return -1/(phi_thr - phi_max) * (phi_rel - phi_max) + 0.5
        elif phi_max < phi_rel <= phi_b:
            return -0.5/(phi_max - phi_b) * (phi_rel - phi_b) + 1.0      
        elif phi_b < phi_rel <= phi_min:
            return 0.5/(phi_b - phi_min) * (phi_rel - phi_min) + 0.5
        elif phi_min < phi_rel <= -phi_min:
            return 1.0/(phi_min - (-phi_min)) * (phi_rel - 0) 
        else:
            return -0.5
        
    def Reward_LT(self,CP_angle_deg,Leg_Num):

        if Leg_Num == 2:
            CP_angle_deg = -CP_angle_deg  # Reflect across the y-axis

        if -180 <= CP_angle_deg <= 0:
            return -np.sin(np.radians(CP_angle_deg))
        elif 0 < CP_angle_deg <= 180:
            return -0.5/180 * CP_angle_deg
        
    def Reward_GravityMoment(self,CP_Angle,Leg_Num):

        if Leg_Num == 2:
            CP_Angle = -CP_Angle  # Reflect across the y-axis

        return -np.sin(np.radians(CP_Angle))

    def _get_pose(self,x,z,phi):

        L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)
       

        ## MODEL CoG
        CG = np.array([x,z])

        ## LEG COORDS
        L1 = np.array([ L*np.sin(gamma_rad),-L*np.cos(gamma_rad)])
        L2 = np.array([-L*np.sin(gamma_rad),-L*np.cos(gamma_rad)])

        ## PROP COORDS
        Prop1 = np.array([ self.PD,0])
        Prop2 = np.array([-self.PD,0])

        ## CONVERT BODY COORDS TO WORLD COORDS
        L1 = CG + self.R_BW(L1,phi)
        L2 = CG + self.R_BW(L2,phi)
        Prop1 = CG + self.R_BW(Prop1,phi)
        Prop2 = CG + self.R_BW(Prop2,phi)

        return np.array([CG,L1,L2,Prop1,Prop2])
    
    def R_BW(self,vec,phi):

        R_BW = np.array([
            [ np.cos(phi), np.sin(phi)],
            [-np.sin(phi), np.cos(phi)],
        ])

        return R_BW.dot(vec)
    
    def R_WP(self,vec,theta):

        R_WP = np.array([
            [ np.cos(theta),-np.sin(theta)],
            [ np.sin(theta), np.cos(theta)]
        ])

        return R_WP.dot(vec)
    
    def R_PW(self,vec,theta):

        R_PW = np.array([
            [ np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        return R_PW.dot(vec)
    
    def R_PC1(self,vec,Beta1):

        R_PC1 = np.array([
            [ np.cos(Beta1),-np.sin(Beta1)],
            [ np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_PC1.dot(vec)
    
    def R_C1P(self,vec,Beta1):

        R_C1P = np.array([
            [ np.cos(Beta1), np.sin(Beta1)],
            [-np.sin(Beta1), np.cos(Beta1)]
        ])

        return R_C1P.dot(vec)
    
    def R_C1B(self,vec,gamma):

        R_C1B = np.array([
            [ np.sin(gamma), np.cos(gamma)],
            [-np.cos(gamma), np.sin(gamma)],
        ])

        return R_C1B.dot(vec)

    def R_PC2(self,vec,Beta2):

        R_PC2 = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)]
        ])

        return R_PC2.dot(vec)
    
    def R_C2P(self,vec,Beta2):

        R_C2P = np.array([
            [ np.cos(Beta2), np.sin(Beta2)],
            [-np.sin(Beta2), np.cos(Beta2)],
        ])

        return R_C2P.dot(vec)

    def R_C2B(self,vec,gamma):

        R_C2B = np.array([
            [-np.sin(gamma), np.cos(gamma)],
            [-np.cos(gamma),-np.sin(gamma)],
        ])

        return R_C2B.dot(vec)




if __name__ == '__main__':
    # Create an instance of the interactive plot and show it
    plot = InteractivePlot()
    plot.show()