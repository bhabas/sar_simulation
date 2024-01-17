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

import plotly.graph_objects as go
from Policy_Training.Policy_Training import Policy_Trainer

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/sar_simulation/crazyflie_projects/Policy_Mapping"

if __name__ == "__main__":


    ## LOAD DATA
    df = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_DR/NL_LR_Trials_DR.csv").dropna() # Collected data
    # df = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_Raw/NL_LR_Trials_Raw.csv").dropna() # Collected data

    ## ORGANIZE DATA
    Tau = df["Tau_Rot_mean"]
    OF_y = df["OFy_Rot_mean"]
    d_ceil = df["D_ceil_Rot_mean"]
    landing_rate = df["LR_4leg"]
    My = df["My_mean"]

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

     ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")


    X_1 = df.query('LR_4leg >= 0.1')['OFy_Rot_mean']
    Y_1 = df.query('LR_4leg >= 0.1')['Tau_Rot_mean']
    Z_1 = df.query('LR_4leg >= 0.1')['D_ceil_Rot_mean']
    C_1 = df.query('LR_4leg >= 0.1')['LR_4leg']

    X_2 = df.query('LR_4leg <= 0.1')['OFy_Rot_mean']
    Y_2 = df.query('LR_4leg <= 0.1')['Tau_Rot_mean']
    Z_2 = df.query('LR_4leg <= 0.1')['D_ceil_Rot_mean']
    C_2 = df.query('LR_4leg <= 0.1')['LR_4leg']



    cmap = mpl.cm.plasma
    norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
    ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)
    cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
    cbar.set_label(label='Landing Success Rate (%)',size=12)

    cbar.ax.tick_params(labelsize=12)


    # PLOT LIMITS AND INFO
    ax.set_xlabel(r'$OF_y \ \mathrm{(rad/s)}$',Fontsize=13)
    ax.set_ylabel(r'$Tau \ \mathrm{(ss)}$',Fontsize=13)
    ax.set_zlabel(r'$d_{ceiling} \ \mathrm{(m)}$',Fontsize=13)



    ax.set_xlim(-20,0)
    ax.set_xticks([-20,-15,-10,-5,0])
    
    ax.tick_params(axis='both', which='major', labelsize=12)
    
    ax.set_ylim(0.35,0.15)
    ax.set_yticks([0.35,0.3,0.25,0.2,0.15])

    ax.set_zlim(0,1.2)
    ax.set_zticks([0,0.2,0.4,0.6,0.8,1.0,1.2])

    ax.view_init(elev=11, azim=-60)
    fig.tight_layout()


    # plt.savefig(f'{model_config}_Policy_Volume_LR.pdf',dpi=300)
    plt.show()
