import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button, TextBox

import matplotlib.patches as patches

import matplotlib.gridspec as gridspec



class InteractivePlot:
    def __init__(self):

        ## INITIAL VALUES
        gamma_deg = 45
        gamma_rad = np.deg2rad(gamma_deg)         
        self.Contact_Leg = 1
        self.L_norm = 1.4
        self.Plane_Angle_deg = 90

        ## CONFIGS
        self.Show_Vel_vec = True
        self.Show_grav_vec = True
        self.Rot_Dir = -1

        ## SAR DIMENSIONAL CONSTRAINTS
        self.PD = 75e-3             # Prop Distance from COM [m]

        self.ax_layout()
        self.init_plots()
        self.Update_1(None) 

    def ax_layout(self):

        self.fig = plt.figure(figsize=(14,10),constrained_layout=False)
        widths = [60,40]
        heights = [80,20]
        gs = self.fig.add_gridspec(ncols=len(widths), nrows=len(heights), width_ratios=widths, height_ratios=heights)

        ## QUAD AX
        widths = [3,97]
        heights = [97,3]
        gs_Quad = gridspec.GridSpecFromSubplotSpec(2, 2, subplot_spec=gs[0,0],width_ratios=widths, height_ratios=heights)
        self.ax_Quad            = self.fig.add_subplot(gs_Quad[0,1])
        self.ax_Gamma_Slider    = self.fig.add_subplot(gs_Quad[0,0])
        self.ax_Leg_Slider      = self.fig.add_subplot(gs_Quad[1,1])

        ## REWARD PLOTS
        widths = [95,5]
        heights = [33,33,33]
        gs_Reward = gridspec.GridSpecFromSubplotSpec(3, 2, subplot_spec=gs[0,1],width_ratios=widths, height_ratios=heights,hspace=0.7,wspace=0)
        self.R_LT_ax            = self.fig.add_subplot(gs_Reward[0,0])
        self.R_GM_ax            = self.fig.add_subplot(gs_Reward[1,0])
        self.R_Phi_ax           = self.fig.add_subplot(gs_Reward[2,0])
        ax_R_LT_Button          = self.fig.add_subplot(gs_Reward[0,1])
        ax_R_GM_Button          = self.fig.add_subplot(gs_Reward[1,1])
        ax_R_Phi_Button         = self.fig.add_subplot(gs_Reward[2,1])




        ## SLIDERS
        gs_Sliders = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=gs[1,1],hspace=1.0)
        self.ax_Plane_Slider    = self.fig.add_subplot(gs_Sliders[0,0])
        self.ax_Beta_Slider     = self.fig.add_subplot(gs_Sliders[1,0])
        self.ax_V_Angle_Slider  = self.fig.add_subplot(gs_Sliders[2,0])


        ## CONFIGS
        widths = [20,80]
        heights = [100]
        gs_Configs = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=gs[1,0],width_ratios=widths, height_ratios=heights)
        gs_Buttons = gridspec.GridSpecFromSubplotSpec(4, 1, subplot_spec=gs_Configs[0,0], wspace=0.5, hspace=0.5)
        gs_Text    = gridspec.GridSpecFromSubplotSpec(3, 2, subplot_spec=gs_Configs[0,1], wspace=0.5, hspace=0.5)


        ax_Leg_Button           = self.fig.add_subplot(gs_Buttons[0, 0])
        ax_Vel_Button           = self.fig.add_subplot(gs_Buttons[1, 0])
        ax_Gravity_Button       = self.fig.add_subplot(gs_Buttons[2, 0])
        ax_Rot_Dir_Button       = self.fig.add_subplot(gs_Buttons[3, 0])
        
        ax_Phi_text             = self.fig.add_subplot(gs_Text[0, 0])
        ax_Phi_rel_text         = self.fig.add_subplot(gs_Text[1, 0])
        ax_Phi_temp_text        = self.fig.add_subplot(gs_Text[2, 0])
        ax_Impact_Window_text   = self.fig.add_subplot(gs_Text[0, 1])
        ax_Temp_GV_text         = self.fig.add_subplot(gs_Text[1, 1])
        ax_Reset_Phi_Button     = self.fig.add_subplot(gs_Text[2, 1])





        ## QUAD SLIDERS
        self.Gamma_Slider = Slider(
            ax=self.ax_Gamma_Slider,
            label="Gamma [deg]",
            valmin=0,
            valmax=90,
            valinit=30,
            valstep=1,
            orientation="vertical"
        )
        self.Gamma_Slider.on_changed(self.Update_1)

        self.Leg_Slider = Slider(
            ax=self.ax_Leg_Slider,
            label='L_norm [m]',
            valmin=0.1,
            valmax=4,
            valinit=self.L_norm,
            valstep=0.1
        )
        self.Leg_Slider.on_changed(self.Update_1)


        ## PLANE SLIDER
        self.Plane_Angle_Slider = Slider(
            ax=self.ax_Plane_Slider,
            label='Plane Angle \n [deg]',
            valmin=0,
            valmax=270,
            valinit=self.Plane_Angle_deg,
            valstep=1
        )
        self.Plane_Angle_Slider.on_changed(self.Update_1) 

        ## BETA SLIDER
        self.Beta_Slider = Slider(
            ax=self.ax_Beta_Slider,
            label='Beta Angle\n [deg]',
            valmin=-720,
            valmax=720,
            valinit=0,
            valstep=1
        )
        self.Beta_Slider.on_changed(self.Update_2)  

        ## FLIGHT ANGLE SLIDER
        self.V_Angle_Slider = Slider(
            ax=self.ax_V_Angle_Slider,
            label='V Angle \n [deg]',
            valmin=0,
            valmax=180,
            valinit=0,
            valstep=1
        )
        self.V_Angle_Slider.valmax = 175
        self.V_Angle_Slider.valmin = 5
        self.V_Angle_Slider.on_changed(self.Update_2) 


        ## BUTTONS
        self.leg_button = Button(ax_Leg_Button, 'Switch Leg', hovercolor='0.975')
        self.leg_button.on_clicked(self.Switch_Leg)

        self.vel_button = Button(ax_Vel_Button, 'Vel. Vector', hovercolor='0.975')
        self.vel_button.on_clicked(self.Vel_Visible)

        self.gravity_button = Button(ax_Gravity_Button, 'Grav. Vector', hovercolor='0.975')
        self.gravity_button.on_clicked(self.Grav_Visible)

        self.Rot_Dir_button = Button(ax_Rot_Dir_Button, f'Rot. Dir: {self.Rot_Dir:d}', hovercolor='0.975')
        self.Rot_Dir_button.on_clicked(self.Change_Rot_Dir)

        self.Reset_Phi_button = Button(ax_Reset_Phi_Button, f'Reset_Phi', hovercolor='0.975')
        self.Reset_Phi_button.on_clicked(self.Reset_Phi)
        


        self.R_LT_Button = Button(ax_R_LT_Button, '', hovercolor='0.975')
        self.R_LT_Button.on_clicked(self.R_LT_Plot_Visible)

        self.R_GM_button = Button(ax_R_GM_Button, '', hovercolor='0.975')
        self.R_GM_button.on_clicked(self.R_GM_Plot_Visible)

        self.R_Phi_button = Button(ax_R_Phi_Button, '', hovercolor='0.975')
        self.R_Phi_button.on_clicked(self.R_Phi_Plot_Visible)


        ## TEXT BOXES
        ax_Phi_text.axis('off')
        self.Phi_text = ax_Phi_text.text(0,0.5,f"Phi: {0.0: 3.1f} [deg]")

        ax_Phi_rel_text.axis('off')
        self.Phi_rel_text = ax_Phi_rel_text.text(0,0.5,f"Phi_rel: {0.0: 3.1f} [deg]")

        ax_Phi_temp_text.axis('off')
        self.Phi_temp_text = ax_Phi_temp_text.text(0,0.5,f"Phi_temp: {0.0: 3.1f} [deg]")

        ax_Impact_Window_text.axis('off')
        self.Impact_Window_text = ax_Impact_Window_text.text(0,0.5,f"Impact Window: {0.0:3.1f} [deg]")

        ax_Temp_GV_text.axis('off')
        self.Temp_GV_text = ax_Temp_GV_text.text(0,0.5,f"(g x v): {0.0: 3.1f} [deg]")

    def init_plots(self):

        ## GEOMETRIC CONSTRAINTS
        L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)

        
        ## QUAD PLOT
        self.ax_Quad.set_aspect('equal', 'box')
        self.ax_Quad.set_xlim(-0.5,0.5)
        self.ax_Quad.set_ylim(-0.5,0.5)

        
        # QUAD LINES
        CG,L1,L2,Prop1,Prop2 = self._get_pose(0,0,0)
        CG_Marker = patches.Circle(CG,radius=0.008,fc='tab:blue',zorder=10)
        self.CG_Marker = self.ax_Quad.add_patch(CG_Marker)

        self.leg1_line,  = self.ax_Quad.plot([],[],lw=2,zorder=5,c='red')
        self.leg2_line,  = self.ax_Quad.plot([],[],lw=2,zorder=5,c='black')
        self.prop1_line, = self.ax_Quad.plot([],[],lw=2,zorder=5,c='black')
        self.prop2_line, = self.ax_Quad.plot([],[],lw=2,zorder=5,c='black')
        self.Z_B_line,   = self.ax_Quad.plot([],[],lw=3,zorder=5,c="tab:blue")
        self.X_B_line,   = self.ax_Quad.plot([],[],lw=3,zorder=5,c="tab:green")


        Swing_Arc = patches.Arc(
            xy=CG,width=L*2,
            height=L*2,
            color="tab:gray",
            zorder=2,
            linestyle="--"
        )
        self.Swing_Arc = self.ax_Quad.add_patch(Swing_Arc)

        # PLANE LINES
        self.Plane_line, = self.ax_Quad.plot([],[],lw=1,zorder=1,c="k")
        self.n_p_line,   = self.ax_Quad.plot([],[],lw=3,zorder=2,c="tab:blue")
        self.t_x_line,   = self.ax_Quad.plot([],[],lw=3,zorder=2,c="tab:green")

        # VECTOR LINES
        self.Vel_line, = self.ax_Quad.plot([],[],c="tab:green", lw=2,zorder=8)
        self.Vel_traj_line, = self.ax_Quad.plot([],[],c="tab:grey",linestyle='--',lw=1,zorder=1,alpha=0.7)
        self.Grav_line, = self.ax_Quad.plot([],[],c="tab:purple", lw=2,zorder=5)

        ## MOMENTUM TRANSFER PLOT
        x = np.linspace(-180,180,500)
        R_Leg1 = np.vectorize(self.Reward_LT)(x,Leg_Num=1)
        R_Leg2 = np.vectorize(self.Reward_LT)(x,Leg_Num=2)
        self.R_LT_Line1, = self.R_LT_ax.plot(x,R_Leg1,label="Leg 1",alpha=1.0)
        self.R_LT_Line2, = self.R_LT_ax.plot(x,R_Leg2,label="Leg 2",alpha=0.2)
        self.R_LT_dot, = self.R_LT_ax.plot([],[],'ro')
        self.R_LT_ax.legend(loc="lower right",ncol=2)
        self.R_LT_ax.grid()

        # CONFIG
        self.R_LT_ax.set_xlim(-200,200)
        self.R_LT_ax.set_ylim(-0.75,1.25)
        self.R_LT_ax.set_xticks(np.arange(-180,181,45))
        self.R_LT_ax.set_yticks(np.arange(-1,1.1,0.5))
        self.R_LT_ax.set_title("Reward: Angular Momentum Transfer")
        self.R_LT_ax.set_xlabel("Angle: (v x e_r)")
        self.R_LT_ax.set_ylabel("Reward")
        self.R_LT_ax.hlines(0,-300,300)
        self.R_LT_ax.vlines(0,-5,5)



        ## GRAVITY MOMENT PLOT
        x = np.linspace(-180,180,500)
        R_Leg1 = np.vectorize(self.Reward_GravityMoment)(x,Leg_Num=1)
        R_Leg2 = np.vectorize(self.Reward_GravityMoment)(x,Leg_Num=2)
        self.R_GM_Line1, = self.R_GM_ax.plot(x,R_Leg1,label="Leg 1",alpha=1.0)
        self.R_GM_Line2, = self.R_GM_ax.plot(x,R_Leg2,label="Leg 2",alpha=0.2)
        self.R_GM_dot, = self.R_GM_ax.plot([],[],'ro')
        self.R_GM_ax.legend(loc="lower right",ncol=2)
        self.R_GM_ax.grid()

        # CONFIG
        self.R_GM_ax.set_xlim(-200,200)
        self.R_GM_ax.set_ylim(-1.25,1.25)
        self.R_GM_ax.set_xticks(np.arange(-180,181,45))
        self.R_GM_ax.set_yticks(np.arange(-1,1.1,0.5))
        self.R_GM_ax.set_title("Reward: Gravity Moment")
        self.R_GM_ax.set_xlabel("Angle: (g x e_r)")
        self.R_GM_ax.set_ylabel("Reward")
        self.R_GM_ax.hlines(0,-300,300)
        self.R_GM_ax.vlines(0,-5,5)
        
        ## IMPACT ANGLE PLOT
        x = np.linspace(-300,300,500)
        Phi_min = -135
        R_dir_neg = np.vectorize(self.Reward_ImpactAngle)(x,Phi_min,rot_dir=-1)
        R_dir_pos = np.vectorize(self.Reward_ImpactAngle)(x,Phi_min,rot_dir= 1)
        self.R_phi_Line1, = self.R_Phi_ax.plot(x,R_dir_neg,label="sign(g x v) < 0")
        self.R_phi_Line2, = self.R_Phi_ax.plot(x,R_dir_pos,label="sign(g x v) > 0")
        self.R_phi_dot, = self.R_Phi_ax.plot([],[],'ro')
        self.R_Phi_ax.legend(loc="lower right",ncol=2)
        self.R_Phi_ax.grid()

        # CONFIG
        self.R_Phi_ax.set_xlim(-270,270)
        self.R_Phi_ax.set_ylim(-1.25,1.25)
        self.R_Phi_ax.set_xticks(np.arange(-225,225+1,45))
        self.R_Phi_ax.set_yticks(np.arange(-1,1.1,0.5))
        self.R_Phi_ax.set_title("Reward: Impact Angle Window")
        self.R_Phi_ax.set_xlabel("Angle: Phi_rel")
        self.R_Phi_ax.set_ylabel("Reward")
        self.R_Phi_ax.hlines(0,-300,300)
        self.R_Phi_ax.vlines(0,-5,5)

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

            ## CALC BETA BOUNDARY ANGLE
            self.Beta_min_rad = -self.Beta_min_rad # Swap sign to match coordinate notation
            self.Beta_min_deg = np.rad2deg(self.Beta_min_rad)

            # self.Beta_Slider.valmax = self.Beta_min_deg
            # self.Beta_Slider.valmin = -(90 + gamma_deg)

            ## CALC PHI BOUNDARY ANGLE
            self.Phi_Impact_min_rad = np.arctan2(-np.cos(self.Beta_min_rad + gamma_rad + Plane_Angle_rad), \
                                                  np.sin(self.Beta_min_rad + gamma_rad + Plane_Angle_rad))
            self.Phi_Impact_min_deg = np.rad2deg(self.Phi_Impact_min_rad)

            ## CALC RELATIVE PHI BOUNDARY ANGLE
            self.Phi_rel_Impact_min_rad = np.arctan2(np.sin(self.Phi_Impact_min_rad - Plane_Angle_rad), \
                                                     np.cos(self.Phi_Impact_min_rad - Plane_Angle_rad))
            self.Phi_rel_Impact_min_deg = np.rad2deg(self.Phi_rel_Impact_min_rad)



        elif self.Contact_Leg == 2:

            ## CALC BETA BOUNDARY ANGLE
            self.Beta_min_rad = -(np.pi - self.Beta_min_rad)
            self.Beta_min_deg = np.rad2deg(self.Beta_min_rad)

            # self.Beta_Slider.valmax = -(90 - gamma_deg)
            # self.Beta_Slider.valmin = self.Beta_min_deg

            ## CALC PHI BOUNDARY ANGLE
            self.Phi_Impact_min_rad = np.arctan2(-np.cos(self.Beta_min_rad - gamma_rad + Plane_Angle_rad), \
                                                  np.sin(self.Beta_min_rad - gamma_rad + Plane_Angle_rad))
            self.Phi_Impact_min_deg = np.rad2deg(self.Phi_Impact_min_rad)

            ## CALC RELATIVE PHI BOUNDARY ANGLE
            self.Phi_rel_Impact_min_rad = np.arctan2(np.sin(self.Phi_Impact_min_rad - Plane_Angle_rad), \
                                                     np.cos(self.Phi_Impact_min_rad - Plane_Angle_rad))
            self.Phi_rel_Impact_min_deg = np.rad2deg(self.Phi_rel_Impact_min_rad)


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
        Vel_angle = np.radians(self.V_Angle_Slider.val)
        V_B_P = np.array([np.cos(Vel_angle),np.sin(Vel_angle)])
        V_B_O = self.R_PW(V_B_P,Plane_Angle_rad)
        V_hat = V_B_O

        ## GRAVITY VECTOR
        g_hat = np.array([0,-1]) # {X_W,Z_W}
        Rot_Dir = -np.sign(-np.cross(g_hat,V_hat))




        if self.Contact_Leg == 1:

            Phi_rad = (Beta_rad + gamma_rad + Plane_Angle_rad) - np.pi/2
            Phi_deg = np.rad2deg(Phi_rad)

            Phi_rel_rad = -(Phi_rad - Plane_Angle_rad)
            Phi_rel_deg = np.rad2deg(Phi_rel_rad)


            Phi_temp_rad = Phi_rel_rad
            Phi_temp_deg = np.rad2deg(Phi_temp_rad)
            

            Phi_rel_norm_rad = Phi_rel_rad
            Phi_rel_norm_deg = np.rad2deg(Phi_rel_norm_rad)



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

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_rel_deg,self.Phi_rel_Impact_min_deg,Rot_Dir)
            
           

        elif self.Contact_Leg == 2:

            Phi_rad = (Beta_rad - gamma_rad + Plane_Angle_rad) - np.pi/2
            Phi_deg = np.rad2deg(Phi_rad)

            Phi_rel_rad = -(Phi_rad - Plane_Angle_rad)
            Phi_rel_deg = np.rad2deg(Phi_rel_rad)


            Phi_temp_rad = Phi_rel_rad
            Phi_temp_deg = np.rad2deg(Phi_temp_rad)
            

            Phi_rel_norm_rad = Phi_rel_rad
            Phi_rel_norm_deg = np.rad2deg(Phi_rel_norm_rad)


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

            ## PHI IMPACT REWARD
            R_Phi = self.Reward_ImpactAngle(Phi_rel_deg,self.Phi_rel_Impact_min_deg,Rot_Dir)

            
        


        ## UPDATE BODY DRAWING
        CG,L1,L2,Prop1,Prop2 = self._get_pose(r_B_O[0],r_B_O[1],Phi_rad)
        self.leg1_line.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2_line.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1_line.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2_line.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])
        self.CG_Marker.set_center(CG)


        X_B = 0.03*np.array([1,0])         # {X_B,Z_B}
        X_B = self.R_BW(X_B,Phi_rad)    # {X_W,Z_W}
        self.X_B_line.set_data([CG[0],CG[0]+X_B[0]],[CG[1],CG[1]+X_B[1]])

        Z_B = 0.03*np.array([0,1])         # {X_B,Z_B}
        Z_B = self.R_BW(Z_B,Phi_rad)    # {X_W,Z_W}
        self.Z_B_line.set_data([CG[0],CG[0]+Z_B[0]],[CG[1],CG[1]+Z_B[1]])



        

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

        self.Phi_text.set_text(f"Phi: {Phi_deg: 3.1f} [deg]")
        self.Phi_rel_text.set_text(f"Phi_rel: {Phi_rel_deg: 3.1f} [deg]")
        self.Phi_temp_text.set_text(f"Phi_temp: {Phi_temp_deg: 3.1f} [deg]")


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

        ## IMPACT ANGLE REWARD
        x = np.linspace(-300,300,500)
        R_dir_neg = np.vectorize(self.Reward_ImpactAngle)(x,self.Phi_rel_Impact_min_deg,rot_dir=-1)
        R_dir_pos = np.vectorize(self.Reward_ImpactAngle)(x,self.Phi_rel_Impact_min_deg,rot_dir= 1)

        if Rot_Dir >= 0:
            self.R_phi_Line1.set_alpha(0.2)
            self.R_phi_Line2.set_alpha(1.0)
        elif Rot_Dir < 0:
            self.R_phi_Line1.set_alpha(1.0)
            self.R_phi_Line2.set_alpha(0.2)


        self.R_phi_Line1.set_data(x,R_dir_neg)
        self.R_phi_Line2.set_data(x,R_dir_pos)
        self.R_phi_dot.set_data(Phi_rel_deg,R_Phi)


        self.fig.canvas.draw_idle()






    def Switch_Leg(self,event):

        if self.Contact_Leg == 1:
            self.Contact_Leg = 2

            ## UPDATE LT ALPHA
            self.R_LT_Line1.set_alpha(0.2)
            self.R_LT_Line2.set_alpha(1.0)

            ## UPDATE GM ALPHA
            self.R_GM_Line1.set_alpha(0.2)
            self.R_GM_Line2.set_alpha(1.0)

        elif self.Contact_Leg == 2:
            self.Contact_Leg = 1

            ## UPDATE LT ALPHA
            self.R_LT_Line1.set_alpha(1.0)
            self.R_LT_Line2.set_alpha(0.2)

            ## UPDATE GM ALPHA
            self.R_GM_Line1.set_alpha(1.0)
            self.R_GM_Line2.set_alpha(0.2)

        self.Update_1(None)

    def Vel_Visible(self,event):

        if self.Show_Vel_vec == True:
            self.Show_Vel_vec = False

        else:
            self.Show_Vel_vec = True

        self.Update_2(None)

    def Grav_Visible(self,event):
        
        if self.Show_grav_vec == True:
            self.Show_grav_vec = False
        else:
            self.Show_grav_vec = True

        self.Update_2(None)

    def Change_Rot_Dir(self,event):

        if self.Rot_Dir >= 0:
            self.Rot_Dir = -1
            self.Rot_Dir_button.label.set_text(f'Rot. Dir: -1')

        elif self.Rot_Dir < 0:
            self.Rot_Dir = +1
            self.Rot_Dir_button.label.set_text(f'Rot. Dir: +1')

    def Reset_Phi(self,event):

        ## READ GAMMA
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)

        ## READ PLANE ANGLE VALUE
        Plane_Angle_deg = self.Plane_Angle_Slider.val
        Plane_Angle_rad = np.radians(Plane_Angle_deg)

        phi_rad = 0

        if self.Contact_Leg == 1:
            Beta_rad = np.arctan2(np.cos(gamma_rad - phi_rad + Plane_Angle_rad), \
                                  np.sin(gamma_rad - phi_rad + Plane_Angle_rad))
            self.Beta_Slider.val = np.degrees(Beta_rad)
            
        elif self.Contact_Leg == 2:
            Beta_rad = np.arctan2( np.cos(gamma_rad + phi_rad - Plane_Angle_rad), \
                                  -np.sin(gamma_rad + phi_rad - Plane_Angle_rad))
            self.Beta_Slider.val = np.degrees(Beta_rad)

        self.Update_2(None)


    def R_LT_Plot_Visible(self,event):

        self.R_LT_ax.set_visible(not self.R_LT_ax.get_visible())
        plt.draw()

    def R_GM_Plot_Visible(self,event):

        self.R_GM_ax.set_visible(not self.R_GM_ax.get_visible())
        plt.draw()

    def R_Phi_Plot_Visible(self,event):

        self.R_Phi_ax.set_visible(not self.R_Phi_ax.get_visible())
        plt.draw()
 
    def show(self):
        plt.show(block=True)

    def Reward_ImpactAngle(self,Phi_rel_deg,Phi_rel_min_deg,rot_dir=-1,Phi_thr=-200):
        
        Phi_rel_min_deg = -np.abs(Phi_rel_min_deg)

        Phi_max = -180
        Phi_b = (Phi_rel_min_deg + Phi_max)/2

        if rot_dir == +1:
            Phi_rel_deg = -Phi_rel_deg

        if Phi_rel_deg <= Phi_thr:
            return -0.5
        elif Phi_thr < Phi_rel_deg <= Phi_max:
            return -1/(Phi_thr - Phi_max) * (Phi_rel_deg - Phi_max) + 0.5
        elif Phi_max < Phi_rel_deg <= Phi_b:
            return -0.5/(Phi_max - Phi_b) * (Phi_rel_deg - Phi_b) + 1.0      
        elif Phi_b < Phi_rel_deg <= Phi_rel_min_deg:
            return 0.5/(Phi_b - Phi_rel_min_deg) * (Phi_rel_deg - Phi_rel_min_deg) + 0.5
        elif Phi_rel_min_deg < Phi_rel_deg <= -Phi_rel_min_deg:
            return 1.0/(Phi_rel_min_deg - (-Phi_rel_min_deg)) * (Phi_rel_deg - 0) 
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