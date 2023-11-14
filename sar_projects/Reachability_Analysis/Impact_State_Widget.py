
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from matplotlib.widgets import Slider, Button, TextBox
import matplotlib.patches as patches
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation


G = 9.81 # Gravity [m/s^2]

class InteractivePlot:
    def __init__(self):

        ## INITIAL VALUES
        self.gamma_deg = 45
        self.L_norm = 1.4
        self.Plane_Angle_deg = 0

        ## SAR DIMENSIONAL CONSTRAINTS
        self.PD = 75e-3             # Prop Distance from COM [m]


        self.ax_layout()
        self.init_plots()
        # self.Update_1() 

    def ax_layout(self):

        self.fig = plt.figure(figsize=(14,10),constrained_layout=False)
        widths = [60,40]
        heights = [80,20]
        gs = self.fig.add_gridspec(ncols=len(widths), nrows=len(heights), width_ratios=widths, height_ratios=heights)

        ## POLAR AX
        widths = [3,97]
        heights = [97,3]
        gs_Polar = gridspec.GridSpecFromSubplotSpec(2, 2, subplot_spec=gs[0,0],width_ratios=widths, height_ratios=heights)
        self.ax_Polar           = self.fig.add_subplot(gs_Polar[0,1])
        self.ax_Gamma_Slider    = self.fig.add_subplot(gs_Polar[0,0])
        self.ax_Leg_Slider      = self.fig.add_subplot(gs_Polar[1,1])

        ## ANIMATION AX
        self.ax_Quad            = self.fig.add_subplot(gs[0,1])

        ## SLIDERS
        gs_Sliders = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=gs[1,1],hspace=1.0)
        self.ax_Plane_Slider    = self.fig.add_subplot(gs_Sliders[0,0])
        self.ax_Phi_rel_Slider  = self.fig.add_subplot(gs_Sliders[1,0])
        self.ax_V_Angle_Slider  = self.fig.add_subplot(gs_Sliders[2,0])


        ## CONFIGS
        widths = [40,60]
        heights = [100]
        gs_Configs = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=gs[1,0],width_ratios=widths, height_ratios=heights)
        gs_Buttons = gridspec.GridSpecFromSubplotSpec(4, 2, subplot_spec=gs_Configs[0,0], wspace=0.5, hspace=0.5)
        gs_Text    = gridspec.GridSpecFromSubplotSpec(3, 3, subplot_spec=gs_Configs[0,1], wspace=0.5, hspace=0.5)

        # ## BUTTONS
        ax_Leg_Button           = self.fig.add_subplot(gs_Buttons[0, 0])


        ## QUAD SLIDERS
        self.Gamma_Slider = Slider(
            ax=self.ax_Gamma_Slider,
            label="Gamma [deg]",
            valmin=0,
            valmax=90,
            valinit=self.gamma_deg,
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
            valmin=-180,
            valmax=225,
            valinit=self.Plane_Angle_deg,
            valstep=15
        )
        self.Plane_Angle_Slider.on_changed(self.Update_1) 

        ## PHI SLIDER
        self.Phi_rel_Slider = Slider(
            ax=self.ax_Phi_rel_Slider,
            label='Phi_rel_impact \n [deg]',
            valmin=-180,
            valmax=225,
            valinit=0,
            valstep=15
        )
        self.Plane_Angle_Slider.on_changed(self.Update_1)


        ## BUTTONS
        self.IVP_button = Button(ax_Leg_Button, 'Update IVP', hovercolor='0.975')
        self.IVP_button.on_clicked(self.animate_IVP)

    def init_plots(self):

        ## GEOMETRIC CONSTRAINTS
        L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)


        ## QUAD PLOT
        self.ax_Quad.set_aspect('equal', 'box')
        self.ax_Quad.set_xlim(-0.5,0.5)
        self.ax_Quad.set_ylim(-0.5,0.5)
        X_W = 0.05*np.array([1,0])
        Z_W = 0.05*np.array([0,1])
        O_W = np.array([-0.45,-0.45])
        self.X_W_line = self.ax_Quad.plot([O_W[0],O_W[0]+X_W[0]],[O_W[1],O_W[1]+X_W[1]],alpha=0.5,lw=3,zorder=5,c="tab:green")
        self.Z_W_line = self.ax_Quad.plot([O_W[0],O_W[0]+Z_W[0]],[O_W[1],O_W[1]+Z_W[1]],alpha=0.5,lw=3,zorder=5,c="tab:blue")

        self.line, = self.ax_Quad.plot([], [], 'o-', lw=2)



    def Update_1(self,event):
        
        ## SAR CONSTRAINTS
        self.PD = 75e-3             # Prop Distance from COM [m]
        self.M = 35.0e-3            # Body Mass [kg]
        self.Iyy = 17.0e-6          # Body Moment of Inertia [kg*m^2]

        ## READ GAMMA, LENGTH
        self.L = self.Leg_Slider.val*self.PD
        gamma_deg = self.Gamma_Slider.val
        gamma_rad = np.radians(gamma_deg)


        self.I_c = self.Iyy + self.M*self.L**2

    def collect_IVP_sol(self,event):

        initial_angles_deg = np.linspace(30,40,2)
        initial_angles_rad = np.radians(initial_angles_deg)
        initial_velocities = np.linspace(3,4,2)

        # LOOP OVER CONDITIONS
        for i, angle in enumerate(initial_angles_rad):
            for j, velocity in enumerate(initial_velocities):
                sol = solve_ivp(self.impact_ODE, [0, 1], [angle, velocity], dense_output=True)

        print()


    def impact_ODE(self,t,y):
        a = self.M*G*self.L/self.I_c

        Plane_Angle_deg = self.Plane_Angle_Slider.val
        Plane_Angle_rad = np.radians(Plane_Angle_deg)

        return [y[1], a * np.cos(y[0])*np.cos(Plane_Angle_rad) + a * np.sin(y[0])*np.sin(Plane_Angle_rad)]
    
    def equation(self, t, y):
        theta, omega = y
        dtheta_dt = omega
        domega_dt = -(G / 0.1) * np.sin(theta)
        return [dtheta_dt, domega_dt]
    
    def solve(self, y0, t_span, dt):
        t = np.arange(t_span[0], t_span[1], dt)
        sol = solve_ivp(self.equation, [t[0], t[-1]], y0, t_eval=t)
        return sol.t, sol.y
    
    def init(self):
        self.line.set_data([], [])
        return self.line,

    def update(self, i):
            length = 0.2
            x = length * np.sin(self.y[0, i])
            y = -length * np.cos(self.y[0, i])
            self.line.set_data([0, x], [0, y])
            return self.line,


    def animate_IVP(self,event):
        
        y0=[np.pi/4, 0]
        t_span=[0, 10]
        dt=0.05

        self.t, self.y = self.solve(y0, t_span, dt)
        ani = FuncAnimation(self.fig, self.update, frames=len(self.t), init_func=self.init, blit=True, interval=20,repeat=False)
        # plt.show()


    def show(self):
        plt.show(block=True)


if __name__ == '__main__':
    # Create an instance of the interactive plot and show it
    plot = InteractivePlot()
    plot.show()