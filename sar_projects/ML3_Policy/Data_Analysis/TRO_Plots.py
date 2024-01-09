## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

## IMPORT PLOTTING MODULES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm
from matplotlib.ticker import (FormatStrFormatter,AutoMinorLocator)
from matplotlib.colors import LinearSegmentedColormap

import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/ML3_Policy"


## LOAD DATA
df = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_LR_Trials.csv") # Collected data
# df = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_Raw/NL_LR_Trials_Raw.csv").dropna() # Collected data

idx = (df.groupby(['Vel_d','Phi_d'])['LR_4Leg'].transform(max)) == df['LR_4Leg']
df = df[idx].reset_index()



def OFa_LR_Plot():
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")


    X_1 = df['Theta_x_trg_mean']
    Y_1 = df['Tau_trg_mean']
    Z_1 = df['D_ceil_trg_mean']
    C_1 = df['LR_4Leg']

    X_2 = df.query('LR_4Leg <= 0.1')['Theta_x_trg_mean']
    Y_2 = df.query('LR_4Leg <= 0.1')['Tau_trg_mean']
    Z_2 = df.query('LR_4Leg <= 0.1')['D_ceil_trg_mean']
    C_2 = df.query('LR_4Leg <= 0.1')['LR_4Leg']



    cmap = mpl.cm.rainbow
    norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
    # ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)
    cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
    cbar.set_label(label='Landing Success Rate (%)',size=12)

    cbar.ax.tick_params(labelsize=12)


    # PLOT LIMITS AND INFO
    ax.set_xlabel(r'$Theta_x \ \mathrm{(rad/s)}$')
    ax.set_ylabel(r'$Tau \ \mathrm{(ss)}$')
    ax.set_zlabel(r'$D_{ceil} \ \mathrm{(m)}$')



    ax.set_xlim(0,15)
    ax.set_xticks([0,5,10,15])
    
    ax.tick_params(axis='both', which='major', labelsize=12)
    
    ax.set_ylim(0.1,0.5)
    ax.set_yticks([0.1,0.2,0.3,0.4])

    ax.set_zlim(0,1.2)
    ax.set_zticks([0,0.4,0.8,1.2])

    ax.view_init(elev=20, azim=-65)
    fig.tight_layout()

    plt.savefig(f'SNL_OFa_LR_Plot.pdf',dpi=300)

def OFa_LR_Shaded_Plot():
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")


    X_1 = df.query('LR_4Leg >= 0.8')['Theta_x_trg_mean']
    Y_1 = df.query('LR_4Leg >= 0.8')['Tau_trg_mean']
    Z_1 = df.query('LR_4Leg >= 0.8')['D_ceil_trg_mean']
    C_1 = df.query('LR_4Leg >= 0.8')['LR_4Leg']

    X_2 = df.query('LR_4Leg <= 0.8')['Theta_x_trg_mean']
    Y_2 = df.query('LR_4Leg <= 0.8')['Tau_trg_mean']
    Z_2 = df.query('LR_4Leg <= 0.8')['D_ceil_trg_mean']
    C_2 = df.query('LR_4Leg <= 0.8')['LR_4Leg']



    cmap = mpl.cm.coolwarm
    norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.9,depthshade=False)
    ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)
    cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
    cbar.set_label(label='Landing Success Rate (%)',size=12)

    cbar.ax.tick_params(labelsize=12)


    # PLOT LIMITS AND INFO
    ax.set_xlabel(r'$Theta_x \ \mathrm{(rad/s)}$',Fontsize=13)
    ax.set_ylabel(r'$Tau \ \mathrm{(ss)}$',Fontsize=13)
    ax.set_zlabel(r'$D_{ceil} \ \mathrm{(m)}$',Fontsize=13)



    ax.set_xlim(0,15)
    ax.set_xticks([0,5,10,15])
    
    ax.tick_params(axis='both', which='major', labelsize=12)
    
    ax.set_ylim(0.1,0.5)
    ax.set_yticks([0.1,0.2,0.3,0.4])

    ax.set_zlim(0,1.2)
    ax.set_zticks([0,0.4,0.8,1.2])

    ax.view_init(elev=20, azim=-65)
    fig.tight_layout()

    plt.savefig(f'SNL_OFa_LR_Shaded_Plot.pdf',dpi=300)


def OFa_aRot_Plot():

    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")


    X_1 = df['Theta_x_trg_mean']
    Y_1 = df['Tau_trg_mean']
    Z_1 = df['D_ceil_trg_mean']
    C_1 = -df['a_Rot_mean']

    color1 = '#fcfb97'
    color2 = '#39d771'
    color3 = '#1e5ec4'


    colors = [(0, color1), (0.5, color2), (1, color3),]
    custom_colormap = LinearSegmentedColormap.from_list("custom_gradient", colors)



    cmap = mpl.cm.terrain_r
    cmap=custom_colormap
    norm = mpl.colors.Normalize(vmin=0,vmax=8.0)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.5,depthshade=False)
    cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
    cbar.set_label(label='a_Rot_mean Rate (%)',size=12)
    cbar.ax.tick_params(labelsize=12)


    # PLOT LIMITS AND INFO
    ax.set_xlabel(r'$Theta_x \ \mathrm{(rad/s)}$')
    ax.set_ylabel(r'$Tau \ \mathrm{(ss)}$')
    ax.set_zlabel(r'$D_{ceil} \ \mathrm{(m)}$')



    ax.set_xlim(0,15)
    ax.set_xticks([0,5,10,15])
    
    ax.tick_params(axis='both', which='major', labelsize=12)
    
    ax.set_ylim(0.1,0.5)
    ax.set_yticks([0.1,0.2,0.3,0.4])

    ax.set_zlim(0,1.2)
    ax.set_zticks([0,0.4,0.8,1.2])

    ax.view_init(azim=4, elev=17)
    fig.tight_layout()

    plt.savefig(f'SNL_OFa_aRot_Plot.pdf',dpi=300)

def Cartesian_LR_Plot():

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    X_1 = df.query('LR_4Leg >= 0.1')['Vx_trg_mean']
    Y_1 = df.query('LR_4Leg >= 0.1')['Vz_trg_mean']
    Z_1 = df.query('LR_4Leg >= 0.1')['D_ceil_trg_mean']
    C_1 = df.query('LR_4Leg >= 0.1')['LR_4Leg']

    X_2 = df.query('LR_4Leg <= 0.8')['Vx_trg_mean']
    Y_2 = df.query('LR_4Leg <= 0.8')['Vz_trg_mean']
    Z_2 = df.query('LR_4Leg <= 0.8')['D_ceil_trg_mean']
    C_2 = df.query('LR_4Leg <= 0.8')['LR_4Leg']





    cmap = mpl.cm.rainbow
    norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
    # ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)
    cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
    cbar.set_label(label='Landing Success Rate (%)',size=12)

    cbar.ax.tick_params(labelsize=12)


    # PLOT LIMITS AND INFO
    # ax.set_xlabel(r'V_x',Fontsize=13)
    # ax.set_ylabel(r'V_z',Fontsize=13)
    ax.set_zlabel(r'$D_{ceil}$',)



    ax.set_xlim(0,4.0)
    ax.set_xticks([0.0,1.0,2.0,3.0,4.0])
    
    ax.tick_params(axis='both', which='major', labelsize=12)
    
    ax.set_ylim(0,4.0)
    ax.set_yticks([0.0,1.0,2.0,3.0,4.0])

    ax.set_zlim(0,1.2)
    ax.set_zticks([0,0.4,0.8,1.2])

    ax.view_init(elev=28, azim=-132)
    fig.tight_layout()

    plt.savefig(f'SNL_Cartesian_LR_Plot.pdf',dpi=300)

def Cartesian_LR_Plot_2():


    # Data preparation
    X_1 = df.query('LR_4Leg >= 0.1')['Vx_trg_mean']
    Y_1 = df.query('LR_4Leg >= 0.1')['Vz_trg_mean']
    Z_1 = df.query('LR_4Leg >= 0.1')['D_ceil_trg_mean']
    C_1 = df.query('LR_4Leg >= 0.1')['LR_4Leg']

    X_2 = df.query('LR_4Leg <= 0.8')['Vx_trg_mean']
    Y_2 = df.query('LR_4Leg <= 0.8')['Vz_trg_mean']
    Z_2 = df.query('LR_4Leg <= 0.8')['D_ceil_trg_mean']
    C_2 = df.query('LR_4Leg <= 0.8')['LR_4Leg']

    # Initialize figure
    fig = go.Figure()

    # Add traces
    fig.add_trace(go.Scatter3d(x=X_1, y=Y_1, z=Z_1, mode='markers', 
                            marker=dict(size=5, color=C_1, colorscale='Jet', opacity=0.8)))

    # Uncomment below to add the second set of data
    # fig.add_trace(go.Scatter3d(x=X_2, y=Y_2, z=Z_2, mode='markers',
    #                            marker=dict(size=5, color=C_2, colorscale='Jet', opacity=0.25)))

    # Update layout
    fig.update_layout(
        scene = dict(
            xaxis=dict(range=[0, 4.0], title='V_x'),
            yaxis=dict(range=[0, 4.0], title='V_z'),
            zaxis=dict(range=[0, 1.2], title='D_ceil'),
            xaxis_tickvals=[0.0, 1.0, 2.0, 3.0, 4.0],
            yaxis_tickvals=[0.0, 1.0, 2.0, 3.0, 4.0],
            zaxis_tickvals=[0, 0.4, 0.8, 1.2]
        ),
        margin=dict(l=0, r=0, b=0, t=0)
    )

    # Camera angle
    fig.update_layout(scene_camera=dict(eye=dict(x=-1.32, y=-1.32, z=0.28)))

    # Show plot
    fig.show()



if __name__ == "__main__":

      
    # OFa_LR_Plot()
    # OFa_LR_Shaded_Plot()

    OFa_aRot_Plot()
    # Cartesian_LR_Plot()
    

    plt.show(block=True)


    

    # fig = go.Figure()
    # X_grid = np.stack((Tau,OF_y,d_ceil),axis=1)

    # ## PLOT DATA POINTS
    # fig.add_trace(
    #     go.Scatter3d(
    #         ## DATA
    #         x=X_grid[:,1].flatten(),
    #         y=X_grid[:,0].flatten(),
    #         z=X_grid[:,2].flatten(),              

    #         ## MARKER
    #         mode='markers',
    #         marker=dict(
    #             size=3,
    #             color = landing_rate,
    #             cmin = 0.0,
    #             cmax = 1.0,
    #             showscale=False,
    #             colorscale='Plasma',   # choose a colorscale
    #             reversescale=False,
    #             opacity=1.0
    #         )
    #     )
    #     )

    # fig.update_layout(
    #     scene=dict(
    #         xaxis_title='OFy [rad/s]',
    #         yaxis_title='Tau [s]',
    #         zaxis_title='D_ceiling [m]',
    #         xaxis_range=[-20,1],
    #         yaxis_range=[0.4,0.1],
    #         zaxis_range=[0,1.2],
    #     ),
    # )


    # name = 'default'
    # # Default parameters which are used when `layout.scene.camera` is not provided
    # camera = dict(
    #     up=dict(x=0, y=0, z=1),
    #     center=dict(x=0, y=0, z=0),
    #     eye=dict(x=1.0, y=1.0, z=1.5)
    # )

    # fig.update_layout(scene_camera=camera, title=name)
    # fig.show()

  
