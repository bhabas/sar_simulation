import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button

import matplotlib.patches as patches

class InteractivePlot:
    def __init__(self):

        ## INITIAL VALUES
        self.gamma = 45             # Leg Angle [m]
        self.LP_Ratio = 1.0

        ## SAR DIMENSIONAL CONSTRAINTS
        self.PD = 75e-3             # Prop Distance from COM [m]
        self.L = self.LP_Ratio*self.PD
        self.M = 35.0e-3            # Body Mass [kg]

        self.Ixx = 15.8e-6          # Body Moment of Inertia [kg*m^2]
        self.Iyy = 17.0e-6          # Body Moment of Inertia [kg*m^2]
        self.Izz = 31.2e-6          # Body Moment of Inertia [kg*m^2]

        I_c = self.Iyy + self.M*self.L**2
        self.params = (np.deg2rad(self.gamma),self.L,self.PD,self.M,self.Iyy,I_c)


        ## SPECIAL CONFIGS
        self.state = (0,0,np.radians(0),0,0,0)

        

        # Create the figure and the line that we will manipulate
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(12, 6))  # Adjust the figsize as needed


        ## PLOT 1
        self.leg1, = self.ax1.plot([0,0],[0,0],'r', lw=2)
        self.leg2, = self.ax1.plot([0,0],[0,0],'k', lw=2)
        self.prop1, = self.ax1.plot([0,0],[0,0],'k', lw=2)
        self.prop2, = self.ax1.plot([0,0],[0,0],'k', lw=2)       


        CG,L1,L2,Prop1,Prop2 = self._get_pose(0,0,0)
        self.leg1.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])
        
        self.ax1.set_xlim(-0.5,0.5)
        self.ax1.set_ylim(-0.5,0.5)
        self.ax1.hlines(0,-5,5)
        self.ax1.set_aspect('equal', 'box')

        ## PLOT 2

        self.leg1_ax2, = self.ax2.plot([0,0],[0,0],'r', lw=2)
        self.leg2_ax2, = self.ax2.plot([0,0],[0,0],'k', lw=2)
        self.prop1_ax2, = self.ax2.plot([0,0],[0,0],'k', lw=2)
        self.prop2_ax2, = self.ax2.plot([0,0],[0,0],'k', lw=2)       


        CG,L1,L2,Prop1,Prop2 = self._get_pose(0,0,0)
        self.leg1_ax2.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2_ax2.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1_ax2.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2_ax2.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])
        
        self.ax2.set_xlim(-0.5,0.5)
        self.ax2.set_ylim(-0.5,0.5)
        self.ax2.hlines(0,-5,5)
        self.ax2.set_aspect('equal', 'box')

        # adjust the main plot to make room for the sliders
        self.fig.subplots_adjust(left=0.1, right=0.9, bottom=0.25, wspace=0.2)  # wspace controls the space between subplots

        # Adjust the existing sliders
        ax_LP = self.fig.add_axes([0.1, 0.1, 0.35, 0.03])
        ax_gamma = self.fig.add_axes([0.05, 0.25, 0.0225, 0.63])
        ax_phi = self.fig.add_axes([0.58, 0.1, 0.35, 0.03])

        # Make a horizontal slider to control the frequency.
        self.LP_Ratio_Slider = Slider(
            ax=ax_LP,
            label='Leg Prop \n Ratio [m]',
            valmin=0.1,
            valmax=3,
            valinit=1,
            valstep=0.01
        )

        # Make a vertically oriented slider to control the amplitude
        self.gamma_slider = Slider(
            ax=ax_gamma,
            label="Gamma [deg]",
            valmin=0,
            valmax=90,
            valinit=self.gamma,
            valstep=1,
            orientation="vertical"
        )

        # Step 3: Add the new slider for the second plot
        self.phi_slider = Slider(
            ax=ax_phi,
            label='Phi [deg]',
            valmin=-180,
            valmax=0,  # adjust as needed
            valinit=-50,  # adjust as needed
            valstep=1
        )

        # register the update function with each slider
        self.LP_Ratio_Slider.on_changed(self.update)
        self.gamma_slider.on_changed(self.update)
        self.phi_slider.on_changed(self.update2)  # you'd need to define this function


        # Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
        resetax = self.fig.add_axes([0.8, 0.025, 0.1, 0.04])
        self.button = Button(resetax, 'Reset', hovercolor='0.975')
        self.button.on_clicked(self.reset)

    def update(self, val):

        self.gamma = np.radians(self.gamma_slider.val)
        self.L = self.LP_Ratio_Slider.val*self.PD

        ## SOLVE FOR MINIMUM RELATIVE PHI IMPACT ANGLE VIA GEOMETRIC CONSTRAINTS
        a = np.sqrt(self.PD**2 + self.L**2 - 2*self.PD*self.L*np.cos(np.pi/2-self.gamma))
        beta_min = np.arccos((self.L**2 + a**2 - self.PD**2)/(2*a*self.L))
        beta_min = -beta_min # Swap sign to match coordinate notation
        self.phi_min = np.arctan2(-np.cos(beta_min + self.gamma), \
                                              np.sin(beta_min + self.gamma))
        self.phi_min_deg = np.rad2deg(self.phi_min)

        beta = np.arctan2(np.cos(self.gamma - self.phi_min),np.sin(self.gamma-self.phi_min))
        beta_deg = np.rad2deg(beta)
        vec = np.array([-self.PD,0]) # {e_r1,e_beta1}
        x,z = self.R_BW(vec,self.phi_min)

        self.params = (self.gamma,self.L,self.PD,0,0,0)


        CG,L1,L2,Prop1,Prop2 = self._get_pose(x,z,self.phi_min)
        self.leg1.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])

        self.phi_slider.set_val(self.phi_min_deg)
        self.phi_slider.valmax = self.phi_min_deg

        self.fig.canvas.draw_idle()

    def update2(self, val):

        self.gamma = np.radians(self.gamma_slider.val)
        self.L = self.LP_Ratio_Slider.val*self.PD
        phi = np.radians(self.phi_slider.val)       

        beta = np.arctan2(np.cos(self.gamma - phi),np.sin(self.gamma - phi)) 
        beta_deg = np.degrees(beta)

        vec  = np.array([-self.L,0])
        x,z = self.R_C1P(vec,beta)


        CG,L1,L2,Prop1,Prop2 = self._get_pose(x,z,phi)
        self.leg1_ax2.set_data([CG[0],L1[0]],[CG[1],L1[1]])
        self.leg2_ax2.set_data([CG[0],L2[0]],[CG[1],L2[1]])
        self.prop1_ax2.set_data([CG[0],Prop1[0]],[CG[1],Prop1[1]])
        self.prop2_ax2.set_data([CG[0],Prop2[0]],[CG[1],Prop2[1]])

        self.fig.canvas.draw_idle()

        
    def reset(self, event):
        self.LP_Ratio_Slider.reset()
        self.gamma_slider.reset()

    def show(self):
        plt.show()

    def _get_state(self):

        return self.state
    
    def _set_state(self,x,z,phi,vx,vz,dphi):

        self.state = (x,z,phi,vx,vz,dphi)


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