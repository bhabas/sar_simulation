import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button, TextBox

import matplotlib.patches as patches

class InteractivePlot:
    def __init__(self):

        ## INITIAL VALUES
        gamma_deg = 20
        gamma_rad = np.deg2rad(gamma_deg)         
        self.Contact_Leg = 1
        LP_Ratio = 2.0
        Plane_Angle_deg = 0

        ## SAR DIMENSIONAL CONSTRAINTS
        self.PD = 75e-3             # Prop Distance from COM [m]
        L = LP_Ratio*self.PD
        self.params = (gamma_rad,L,self.PD,0,0,0)

        ## CONFIGS
        self.vel_vec = False
        self.grav_vec = False


        self.fig = plt.figure(figsize=(12,6))
        self.ax_Plot = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0,right=0.57,bottom=0.25)

        ## QUADROTOR PLOT 
        CG,L1,L2,Prop1,Prop2 = self._get_pose(0,0,0)
        self.leg1_ax, = self.ax_Plot.plot([CG[0],L1[0]],[CG[1],L1[1]],'r', lw=2)
        self.leg2_ax, = self.ax_Plot.plot([CG[0],L2[0]],[CG[1],L2[1]],'k', lw=2)
        self.prop1_ax, = self.ax_Plot.plot([CG[0],Prop1[0]],[CG[1],Prop1[1]],'k', lw=2)
        self.prop2_ax, = self.ax_Plot.plot([CG[0],Prop2[0]],[CG[1],Prop2[1]],'k', lw=2)

        self.vel_ax, = self.ax_Plot.plot([0,0],[0,0],c="tab:green", lw=2)
        self.grav_ax, = self.ax_Plot.plot([0,0],[0,0],c="tab:purple", lw=2)

        self.Plane_Angle_ax, = self.ax_Plot.plot([-5,5],[0,0],c="k",lw=1,zorder=1)

        ## INITIAL DATA
        self.CG_Marker = patches.Circle(CG,radius=0.01,fc='tab:blue',zorder=10)
        self.ax_CG_Marker = self.ax_Plot.add_patch(self.CG_Marker)

        self.Swing_Arc = patches.Arc(
            xy=CG,width=L*2,
            height=L*2,
            color="tab:gray",
            zorder=2,
            linestyle="--"
        )
        self.ax_Swing_Arc = self.ax_Plot.add_patch(self.Swing_Arc)


        self.ax_Plot.set_xlim(-0.5,0.5)
        self.ax_Plot.set_ylim(-0.5,0.5)
        self.ax_Plot.set_aspect('equal', 'box')


        ## SLIDER AX OBJECTS
        ax_LP_Ratio_Slider = self.fig.add_axes([0.1, 0.1, 0.35, 0.03])
        ax_Gamma_Slider = self.fig.add_axes([0.05, 0.25, 0.0225, 0.63])
        ax_Beta_Slider = self.fig.add_axes([0.58, 0.15, 0.35, 0.03])
        ax_Vel_Slider = self.fig.add_axes([0.58, 0.05, 0.35, 0.03])
        ax_Plane_Angle_Slider = self.fig.add_axes([0.58, 0.25, 0.35, 0.03])


        ## BUTTON AX OBJECTS
        ax_Vel_Button = self.fig.add_axes([0.8, 0.6, 0.15, 0.04])
        self.vel_button = Button(ax_Vel_Button, 'Show Vel Vector', hovercolor='0.975')
        self.vel_button.on_clicked(self.Vel_Visible)

        ax_Gravity_Button = self.fig.add_axes([0.8, 0.5, 0.15, 0.04])
        self.gravity_button = Button(ax_Gravity_Button, 'Show Grav Vector', hovercolor='0.975')
        self.gravity_button.on_clicked(self.Grav_Visible)

        ax_Leg_Button = self.fig.add_axes([0.8, 0.4, 0.15, 0.04])
        self.leg_button = Button(ax_Leg_Button, 'Switch Leg', hovercolor='0.975')
        self.leg_button.on_clicked(self.Switch_Leg)

        # Make a horizontal slider to control the frequency.
        self.LP_Ratio_Slider = Slider(
            ax=ax_LP_Ratio_Slider,
            label='Leg Prop \n Ratio [m]',
            valmin=0.1,
            valmax=4,
            valinit=LP_Ratio,
            valstep=0.1
        )

        # Make a vertically oriented slider to control the amplitude
        self.Gamma_Slider = Slider(
            ax=ax_Gamma_Slider,
            label="Gamma [deg]",
            valmin=0,
            valmax=90,
            valinit=gamma_deg,
            valstep=1,
            orientation="vertical"
        )

        self.Plane_Angle_Slider = Slider(
            ax=ax_Plane_Angle_Slider,
            label='Plane Angle [deg]',
            valmin=0,
            valmax=270,
            valinit=Plane_Angle_deg,
            valstep=1
        )

        self.Beta_Slider = Slider(
            ax=ax_Beta_Slider,
            label='Beta \n [deg]',
            valmin=-360,
            valmax=0,  # adjust as needed
            valinit=0,  # adjust as needed
            valstep=1
        )

        self.Vel_Slider = Slider(
            ax=ax_Vel_Slider,
            label='Flight Angle \n [deg]',
            valmin=-180,
            valmax=180,
            valinit=0,
            valstep=1
        )

        

        ## UPDATE PLOTS
        self.Plane_Angle_Slider.on_changed(self.Min_Angle_Update)  
        self.LP_Ratio_Slider.on_changed(self.Min_Angle_Update)
        self.Gamma_Slider.on_changed(self.Min_Angle_Update)
        self.Beta_Slider.on_changed(self.Beta_Update)
        self.Vel_Slider.on_changed(self.Beta_Update) 

        self.Min_Angle_Update(None) 

    def Min_Angle_Update(self, val):
        
        ## UPDATE PLANE DRAWING
        vec = np.array([5,0])
        vec = self.R_PW(vec,np.radians(self.Plane_Angle_Slider.val))
        self.Plane_Angle_ax.set_data([-vec[0],vec[0]],[-vec[1],vec[1]])

        ## UPDATE GAMMA AND LENGTH
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)
        L = self.LP_Ratio_Slider.val*self.PD
        self.params = (gamma_rad,L,self.PD,0,0,0)
        
        ## UPDATE PLANE ANGLE
        Plane_Angle_deg = self.Plane_Angle_Slider.val
        Plane_Angle_rad = np.radians(Plane_Angle_deg)

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


        ## DRAW ELEMENTS
        self.fig.canvas.draw_idle()

    def Beta_Update(self, val):

        ## READ GAMMA, LENGTH
        L = self.LP_Ratio_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)

        Plane_Angle_deg = self.Plane_Angle_Slider.val
        Plane_Angle_rad = np.radians(Plane_Angle_deg)

        ## UPDATE BETA VALUE
        Beta_deg = self.Beta_Slider.val
        Beta_rad = np.radians(Beta_deg)

        if self.Contact_Leg == 1:
            phi_rad = np.arctan2(-np.cos(Beta_rad + gamma_rad + Plane_Angle_rad), \
                                np.sin(Beta_rad + gamma_rad + Plane_Angle_rad))
            phi_deg = np.rad2deg(phi_rad)

            ## UPDATE BODY POSITION
            r_B_C1 = np.array([-L,0])                                       # {e_r1,e_beta1}
            r_B_C1 = self.R_PW(self.R_C1P(r_B_C1,Beta_rad),Plane_Angle_rad) # {X_W,Z_W}
            r_B_O = r_B_C1   


            ## UPDATE ARC VALUES
            self.Swing_Arc.width = self.Swing_Arc.height = L*2
            self.Swing_Arc.angle = 180-Plane_Angle_deg
            self.Swing_Arc.theta1 = np.abs(self.Beta_min_deg)
            self.Swing_Arc.theta2 = 90 + gamma_deg

        elif self.Contact_Leg == 2:

            phi_rad = np.arctan2(-np.cos(Beta_rad - gamma_rad + Plane_Angle_rad), \
                              np.sin(Beta_rad - gamma_rad + Plane_Angle_rad))
            phi_deg = np.rad2deg(phi_rad)

            r_B_C2 = np.array([-L,0])                                       # {e_r1,e_beta1}
            r_B_C2 = self.R_PW(self.R_C2P(r_B_C2,Beta_rad),Plane_Angle_rad) # {X_W,Z_W}
            r_B_O = r_B_C2

            self.Swing_Arc.width = self.Swing_Arc.height = L*2
            self.Swing_Arc.angle = 180-Plane_Angle_deg
            self.Swing_Arc.theta1 = 90 - gamma_deg
            self.Swing_Arc.theta2 = np.abs(self.Beta_min_deg)
        

        CG,L1,L2,Prop1,Prop2 = self._get_pose(r_B_O[0],r_B_O[1],phi_rad)

        ## UPDATE BODY DRAWING
        self.leg1_ax.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2_ax.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1_ax.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2_ax.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])
        self.CG_Marker.set_center(CG)

        


        ## UPDATE VEL VECTOR
        vel_angle = np.radians(self.Vel_Slider.val)
        if self.vel_vec == True:
            self.vel_ax.set_data([CG[0],CG[0] + 0.1*np.cos(vel_angle)],[CG[1],CG[1] + 0.1*np.sin(vel_angle)])
        else:
            self.vel_ax.set_data([None,None],[None,None])

        ## UPDATE GRAVITY VECTOR
        if self.grav_vec == True:
            self.grav_ax.set_data([CG[0],CG[0]],[CG[1],CG[1]-0.1])
        else:
            self.grav_ax.set_data([None,None],[None,None])

        ## DRAW ELEMENTS
        self.fig.canvas.draw_idle()

    def Vel_Visible(self,event):

        if self.vel_vec == True:
            self.vel_vec = False
        else:
            self.vel_vec = True

        self.Beta_Update(None)

    def Grav_Visible(self,event):

        if self.grav_vec == True:
            self.grav_vec = False
        else:
            self.grav_vec = True

        self.Beta_Update(None)

    def Switch_Leg(self,event):

        if self.Contact_Leg == 1:
            self.Contact_Leg = 2
        else:
            self.Contact_Leg = 1

        self.Min_Angle_Update(None)


    def show(self):
        plt.show()

    def _get_pose(self,x,z,phi):

        gamma,L,PD,M,Iyy,I_c = self.params

        ## MODEL CoG
        CG = np.array([x,z])

        ## LEG COORDS
        L1 = np.array([ L*np.sin(gamma),-L*np.cos(gamma)])
        L2 = np.array([-L*np.sin(gamma),-L*np.cos(gamma)])

        ## PROP COORDS
        Prop1 = np.array([ PD,0])
        Prop2 = np.array([-PD,0])

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