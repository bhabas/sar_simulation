## IMPORT MODULES
import pandas as pd
import numpy as np
from sklearn import linear_model
from scipy.interpolate import griddata
import scipy
import plotly.graph_objects as go
import os,sys

## IMPORT PLOTTING MODULES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm
from matplotlib.ticker import (FormatStrFormatter,AutoMinorLocator)

sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/')
from data_analysis.Data_Analysis import DataFile

os.system("clear")
mpl.rcParams['pdf.fonttype'] = 42
mpl.rcParams['ps.fonttype'] = 42



## RAW DATA FUNCTIONS
def plot_mpl_3d(df,Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if XY_data==None:
        if polar==True:
            X = df['vx']
            Y = df['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df['phi_IC']
            Y = df['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df[XY_data[0]]
        Y = df[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

        
    Z = df[Z_data]
    C = df[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X,Y,Z,c=C,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
    fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_zlabel(Z_data)
    # ax.set_ylim(2,7)
    fig.tight_layout()

    return fig,ax

def plot_plotly_3d(df,Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if XY_data==None:
        if polar==True:
            X = df['vx']
            Y = df['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df['phi_IC']
            Y = df['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df[XY_data[0]]
        Y = df[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

        
    Z = df[Z_data]
    C = df[color_data]
   

    fig = go.Figure(data=[go.Scatter3d(x=X, y=Y, z=Z,
                                   mode='markers',marker=dict(
        size=2,
        color=C,                # set color to an array/list of desired values
        colorscale='Jet',   # choose a colorscale
        opacity=1.0
    ))])
    
    # CREATE PLOTS AND COLOR BAR
    fig.update_layout(
    scene = dict(
        xaxis_title = XY_str[0],
        yaxis_title = XY_str[1],
        zaxis_title = Z_data,
        # xaxis = dict(range=[0,5]),
        # yaxis = dict(range=[0,8]),
        # zaxis = dict(nticks=4, range=[0,360])
        ))

    fig.show()

def plot_scatter_2d(df,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111)

    ## DEFINE PLOT VARIABLES
    if XY_data==None:
        if polar==True:
            X = df['vx']
            Y = df['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df['phi_IC']
            Y = df['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df[XY_data[0]]
        Y = df[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

    C = df[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X,Y,c=C,cmap=cmap,norm=norm,alpha=0.6)
    # fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label=color_str)


    # PLOT LIMITS AND INFO
    ax.grid()
    ax.set_axisbelow(True)
    fig.tight_layout()

    return fig,ax


## OPTIMAL DATA FUNCTIONS
def plot_best(Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if XY_data==None:

        if polar==True:
            X = df['vx']
            Y = df['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df['phi_IC']
            Y = df['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df[XY_data[0]]
        Y = df[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

        
    Z = df[Z_data]
    C = df[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X,Y,Z,c=C,cmap=cmap,norm=norm)
    fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_zlabel(Z_data)
    ax.set_title("Best Data")
    fig.tight_layout()

def plot_best_surface(Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True):
    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if polar==True:
        X = df['vx']
        Y = df['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df['phi_IC']
        Y = df['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df[Z_data]
    C = df[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)


    # CREATE PLOTS AND COLOR BAR
    ax.plot_trisurf(X,Y,Z,cmap=cmap,norm=norm,edgecolors='grey',linewidth=0.25)
    fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_xlabel('X-Velocity (m/s)')
    ax.set_ylabel('Z-Velocity (m/s)')
    ax.set_zlabel(Z_data)
    ax.set_title('Surface Best Data')

def plot_best_surface_smoothed(Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True):
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if polar==True:
        X = df['vx']
        Y = df['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df['phi_IC']
        Y = df['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df[Z_data]
    C = df[color_data]

    ## ADJUST COLOR SCALING AND LABELS
    if color_str == None:
        color_str = color_data

    if color_norm == None:
        vmin = np.min(C)
        vmax = np.max(C)
    else:
        vmin = color_norm[0]
        vmax = color_norm[1]

    # SOMETHING ABOUT DEFINING A GRID
    xi = np.linspace(X.min(),X.max(),(len(Z)//7))
    yi = np.linspace(Y.min(),Y.max(),(len(Z)//7))
    zi = griddata((X, Y), Z, (xi[None,:], yi[:,None]), method='linear')
    xig, yig = np.meshgrid(xi, yi)

    # DEFINE COLOR FORMAT AND LIMITS
    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=vmin,vmax=vmax)

    # CREATE PLOTS AND COLOR BAR
    surf = ax.plot_surface(xig, yig, zi,cmap=cmap,norm=norm,edgecolors='grey',linewidth=0.25)
    # surf = ax.plot_surface(xig, yig, zi,cmap=cmap,norm=norm)
    # surf = ax.contour(xig, yig, zi,cmap=cmap,norm=norm)

    fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label=color_str)

    # PLOT LIMITS AND INFO
    ax.set_zlabel(Z_data)
    ax.set_title('Smoothed Best Data Surface')

def plot_polar_smoothed(df,plotFig=True,saveFig=False):

    # # COLLECT DATA
    R = df.iloc[:]['vel_IC']
    Theta = df.iloc[:]['phi_IC']
    C = df.iloc[:]['landing_rate_4_leg']

    # SOMETHING ABOUT DEFINING A GRID
    interp_factor = 20
    ri = np.linspace(R.min(),R.max(),(len(C)//interp_factor))
    thetai = np.linspace(Theta.min(),Theta.max(),(len(C)//interp_factor))
    r_ig, theta_ig = np.meshgrid(ri, thetai)
    zi = griddata((R, Theta), C, (ri[None,:], thetai[:,None]), method='linear')
    zi = zi + 0.0001
    
    FONT_SIZE = 16

    ## INIT PLOT INFO
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=-0.1,vmax=1)
    
    ax.contourf(np.radians(theta_ig),r_ig,zi,cmap=cmap,norm=norm,levels=20)

    ax.set_thetamin(20)
    ax.set_thetamax(90)

    ax.set_yticks([0.0,1.0,2.0,3.0,4.0])
    ax.set_xticks(np.radians([20,40,60,80,90]))
    ax.tick_params(axis='x',labelsize=FONT_SIZE)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))


    ax.set_rmin(0)
    ax.set_rmax(4)
    ax.tick_params(axis='y',labelsize=FONT_SIZE)

    
    


    ## AXIS LABELS    
    ax.text(np.radians(3),2.5,'Flight Velocity (m/s)',
        rotation=18,ha='center',va='center',fontsize=FONT_SIZE)

    ax.text(np.radians(70),4.4,'Flight Angle (deg)',
        rotation=0,ha='left',va='center',fontsize=FONT_SIZE)

    if saveFig==True:
        plt.savefig(f'{model_config}_Polar_LR.pdf',dpi=300,bbox_inches='tight')

    if plotFig == True:
        plt.show()


def plot_all_eul():

    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    X = df['impact_vx_arr']
    for idx, i in enumerate(X): 
            X[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    X = np.concatenate(X,axis=0)

    Y = df['impact_vz_arr']
    for idx, i in enumerate(Y): 
            Y[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    Y = np.concatenate(Y,axis=0)

    Z = df['impact_eul_arr']
    for idx, i in enumerate(Z): 
            Z[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    Z = np.concatenate(Z,axis=0)

    C = df['contact_arr']
    for idx, i in enumerate(C): 
            C[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    C = np.concatenate(C,axis=0)
   
   
    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=0,vmax=4)
    
    # CREATE PLOTS AND COLOR BAR
    ax.scatter(X[::5],Y[::5],Z[::5],c=C[::5],cmap=cmap,norm=norm)
    fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label='leg contacts')


    # PLOT LIMITS AND INFO
    # ax.set_zlabel(Z_data)
    # ax.set_ylim(0,8)
    ax.set_title("Raw Data")
    fig.tight_layout()


if __name__ == '__main__':

    ## FULL DATAFRAME

    model_config = "Narrow-Short"
    df_raw = pd.read_csv(f"Projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_raw = df_raw.query(f"landing_rate_4_leg >= {0.0}")

    


    ## MAX LANDING RATE DATAFRAME
    idx = df_raw.groupby(['vel_IC','phi_IC'])['landing_rate_4_leg'].transform(max) == df_raw['landing_rate_4_leg']
    df_max = df_raw[idx].reset_index()

    model_list = ['ExtraNarrow-Short','Narrow-Short','Wide-Short','ExtraNarrow-Long','Narrow-Long','Wide-Long']
    
    # for model_config in model_list:
    #     df_raw = pd.read_csv(f"Projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    #     df_raw = df_raw.query(f"landing_rate_4_leg >= {0.0}")

        


    #     ## MAX LANDING RATE DATAFRAME
    #     idx = df_raw.groupby(['vel_IC','phi_IC'])['landing_rate_4_leg'].transform(max) == df_raw['landing_rate_4_leg']
    #     df_max = df_raw[idx].reset_index()

    #     plot_polar_smoothed(df_max,plotFig=False,saveFig=True)


    





    ############################
    #        COLOR BAR
    ############################
    def plot_color_bar():
        ## INITIALIZE FIGURE
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=100)
        
        fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label='Landing Percentage (%) ')
        ax.remove()

        fig.tight_layout()

        plt.savefig(f'Color_Bar_LR.pdf',dpi=300,bbox_inches='tight')
    
    # plot_color_bar()

    ############################
    #     Official Plots
    ############################


    ## LANDING RATE POLAR PLOT
    plot_polar_smoothed(df_max,saveFig=True)
    # plot_mpl_3d(df_max,Z_data='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)


    # FIGURE 1: 3D POLAR (VEL, PHI, D_CEILING, LANDING_RATE)
    # plot_mpl_3d(df_max,Z_data='flip_d_mean',color_data='landing_rate_4_leg',polar=True)
    def plot_fig1():
    
        ## INITIALIZE FIGURE
        fig = plt.figure()
        ax = fig.add_subplot(111,projection="3d")

        df = df_max

        X_1 = df.query('landing_rate_4_leg >= 0.1')['vx']
        Y_1 = df.query('landing_rate_4_leg >= 0.1')['vz']
        Z_1 = df.query('landing_rate_4_leg >= 0.1')['flip_d_mean']
        C_1 = df.query('landing_rate_4_leg >= 0.1')['landing_rate_4_leg']

        X_2 = df.query('landing_rate_4_leg <= 0.1')['vx']
        Y_2 = df.query('landing_rate_4_leg <= 0.1')['vz']
        Z_2 = df.query('landing_rate_4_leg <= 0.1')['flip_d_mean']
        C_2 = df.query('landing_rate_4_leg <= 0.1')['landing_rate_4_leg']



        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
        
        # CREATE PLOTS AND COLOR BAR
        ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
        ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.2,depthshade=False)
        cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
        cbar.set_label(label='Landing Success Rate (%)',size=12)

        cbar.ax.tick_params(labelsize=12)


        # PLOT LIMITS AND INFO
        ax.set_xlabel('X-Velocity (m/s)',size=12)
        ax.set_ylabel('Z-Velocity (m/s)',size=12)
        ax.set_zlabel('d_ceiling (m)',size=12)


        ax.set_xlim(0,4)
        ax.set_xticks([0,1.0,2.0,3.0,4.0])
        
        ax.tick_params(axis='both', which='major', labelsize=12)
        
        ax.set_ylim(0,4)
        ax.set_yticks([0,1.0,2.0,3.0,4.0])

        ax.set_zlim(0,1.0)
        ax.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

        ax.view_init(elev=36, azim=-122)
        fig.tight_layout()


        plt.savefig(f'{model_config}_StateSpace.pdf',dpi=300)
        plt.show()

    plot_fig1()


    # plot_mpl_3d(df_max,Z_data='flip_d_mean',color_data='My_d',polar=True)



    ############################
    #        FIGURE 2
    ############################
    # 2D CARTESIAN (OF_y, RREV, LANDING_RATE)

    def plot_fig2():
        # fig,ax = plot_scatter_2d(df_max,color_data='landing_rate_4_leg',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'))
        
        
        ## INITIALIZE FIGURE
        fig = plt.figure()
        ax = fig.add_subplot(111)
        df = df_max

        ## DEFINE PLOT VARIABLES
        
        

        X_1 = df.query('landing_rate_4_leg >= 0.1')['OF_y_flip_mean']
        Y_1 = df.query('landing_rate_4_leg >= 0.1')['RREV_flip_mean']
        C_1 = df.query('landing_rate_4_leg >= 0.1')['landing_rate_4_leg']

        X_2 = df.query('landing_rate_4_leg <= 0.1')['OF_y_flip_mean']-0.15
        Y_2 = df.query('landing_rate_4_leg <= 0.1')['RREV_flip_mean']
        C_2 = df.query('landing_rate_4_leg <= 0.1')['landing_rate_4_leg']

        
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        
        # CREATE PLOTS AND COLOR BAR
        ax.scatter(X_1,Y_1,c=C_1,cmap=cmap,norm=norm,alpha=0.6)
        ax.scatter(X_2,Y_2,c=C_2,cmap=cmap,norm=norm,alpha=0.1)

        cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
        cbar.ax.tick_params(labelsize=10)
        cbar.set_label(label='Landing Success Rate (%)',size=12)



        # PLOT LIMITS AND INFO
        ax.grid()
        ax.set_axisbelow(True)

        ax.set_xticks([0,-5,-10,-15,-20])
        ax.tick_params(axis='x',labelsize=12)


        ax.set_yticks(np.arange(0,9,1))
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
        ax.tick_params(axis='y',labelsize=12)

        ax.set_xlabel('OF_y (rad/s)',Fontsize=12)
        ax.set_ylabel('RREV (rad/s)',Fontsize=12)


        fig.tight_layout()

        # plt.savefig(f'{model_config}_OF_Space.pdf',dpi=300,bbox_inches='tight')
        plt.show()

    plot_fig2()

    ############################
    #        FIGURE 3
    ############################
    ## 3D CARTESIAN (OF_y, RREV, D_CEILING, LANDING RATE)
    # plot_mpl_3d(df_max,Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg')

    def plot_fig3():
    
        ## INITIALIZE FIGURE
        fig = plt.figure()
        ax = fig.add_subplot(111,projection="3d")

        df = df_max

        X_1 = df.query('landing_rate_4_leg >= 0.1')['OF_y_flip_mean']
        Y_1 = df.query('landing_rate_4_leg >= 0.1')['RREV_flip_mean']
        Z_1 = df.query('landing_rate_4_leg >= 0.1')['flip_d_mean']
        C_1 = df.query('landing_rate_4_leg >= 0.1')['landing_rate_4_leg']

        X_2 = df.query('landing_rate_4_leg <= 0.1')['OF_y_flip_mean']
        Y_2 = df.query('landing_rate_4_leg <= 0.1')['RREV_flip_mean']
        Z_2 = df.query('landing_rate_4_leg <= 0.1')['flip_d_mean']
        C_2 = df.query('landing_rate_4_leg <= 0.1')['landing_rate_4_leg']



        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1.0)
        
        # CREATE PLOTS AND COLOR BAR
        ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
        ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)
        cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
        cbar.set_label(label='Landing Success Rate (%)',size=12)

        cbar.ax.tick_params(labelsize=12)


        # PLOT LIMITS AND INFO
        ax.set_xlabel('OF_y (rad/s)',size=12)
        ax.set_ylabel('RREV (rad/s)',size=12)
        ax.set_zlabel('d_ceiling (m)',size=12)


        ax.set_xlim(-20,0)
        ax.set_xticks([-20,-15,-10,-5,0])
        
        ax.tick_params(axis='both', which='major', labelsize=12)
        
        ax.set_ylim(0,8)
        ax.set_yticks([0,2,4,6,8])

        ax.set_zlim(0,1.0)
        ax.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

        ax.view_init(elev=11, azim=-60)
        fig.tight_layout()


        plt.savefig(f'{model_config}_Policy_Volume_LR.pdf',dpi=300)
        plt.show()

    plot_fig3()
    

    ############################
    #        FIGURE 4
    ############################
    ## 3D CARTESIAN (OF_y, RREV, D_CEILING, LANDING RATE)
    # plot_mpl_3d(df_max,Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg')

    def plot_fig4():
        ## INITIALIZE FIGURE
        fig = plt.figure()
        ax = fig.add_subplot(111,projection="3d")

        df = df_max

        X_1 = df.query('landing_rate_4_leg >= 0.6')['OF_y_flip_mean']
        Y_1 = df.query('landing_rate_4_leg >= 0.6')['RREV_flip_mean']
        Z_1 = df.query('landing_rate_4_leg >= 0.6')['flip_d_mean']
        C_1 = df.query('landing_rate_4_leg >= 0.6')['My_d']

        X_2 = df.query('landing_rate_4_leg <= 0.1')['OF_y_flip_mean']
        Y_2 = df.query('landing_rate_4_leg <= 0.1')['RREV_flip_mean']
        Z_2 = df.query('landing_rate_4_leg <= 0.1')['flip_d_mean']
        C_2 = df.query('landing_rate_4_leg <= 0.1')['My_d']





        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=10.0)
        
        # CREATE PLOTS AND COLOR BAR
        ax.scatter(X_1,Y_1,Z_1,c=C_1,cmap=cmap,norm=norm,alpha=0.8,depthshade=False)
        # ax.scatter(X_2,Y_2,Z_2,c=C_2,cmap=cmap,norm=norm,alpha=0.25,depthshade=False)
        cbar = fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm))
        cbar.set_label(label='M_yd (N*mm)',size=12)

        cbar.ax.tick_params(labelsize=12)


        # PLOT LIMITS AND INFO
        ax.set_xlabel('OF_y (rad/s)',size=12)
        ax.set_ylabel('RREV (rad/s)',size=12)
        ax.set_zlabel('d_ceiling (m)',size=12)


        ax.set_xlim(-20,0)
        ax.set_xticks([-20,-15,-10,-5,0])
        
        ax.tick_params(axis='both', which='major', labelsize=12)
        
        ax.set_ylim(0,8)
        ax.set_yticks([0,2,4,6,8])

        ax.set_zlim(0,1.0)
        ax.set_zticks([0,0.2,0.4,0.6,0.8,1.0])

        ax.view_init(elev=11, azim=-60)
        fig.tight_layout()

        plt.savefig(f'{model_config}_Policy_Volume_My_d.pdf',dpi=300)
        plt.show()

    plot_fig4()        






    # FIGURE 4: 3D CARTESIAN (OF_y, RREV, D_CEILING, My VALUE)
    # plot_mpl_3d(df_raw,Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='My_d')
    # plot_plotly_3d(df_raw.query(f"landing_rate_4_leg >= {0.6}"),Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='My_d')





    ## PLOT POLAR DATAPOINTS
    # plot_raw(df_max,Z_data='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)



    ## RREV VS OF_d VS My_d
    # fig,ax = plot_raw(df_raw,Z_data='My_d',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg')
    # plot_plotly(df_raw,Z_data='My_d',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg')




    ## IMPROVED POLICY IDEA
    # fig,ax = plot_raw(df_raw,Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg',color_norm=(0,1))
    # plot_plotly(df_raw,Z_data='vel_IC',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='My_d')
    # plot_plotly(df_raw,Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='My_d')




    ## PLOT POLICY REGION W/ TRAJECTORY DATA
    # fig,ax = plot_raw(df_raw,Z_data='flip_d_mean',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg',color_norm=(0,1))
    
    # dataPath = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/logs/"
    # fileName = 'EM_PEPG--Vd_3.50--phi_35.00--trial_12--NL.csv'
    # trial = DataFile(dataPath,fileName,dataType='SIM')

    # k_ep = 0
    # k_run = 0
    # t = trial.grab_stateData(k_ep,k_run,['t'])
    # RREV = trial.grab_stateData(k_ep,k_run,['RREV'])
    # OF_y = trial.grab_stateData(k_ep,k_run,['OF_y'])
    # d_ceiling = 2.1-trial.grab_stateData(k_ep,k_run,['z'])
    # ax.plot(OF_y.flatten(),RREV.flatten(),d_ceiling.flatten(),'k--',)

    # ax.set_xlabel('OF_y [rad/s]')
    # ax.set_ylabel('RREV [rad/s]')
    # ax.set_zlabel('d_ceiling [m]')

    # ax.set_xticks([0,-5,-10,-15,-20])
    # ax.set_yticks([0,2,4,6,8,10])
    # ax.set_zticks([0,0.5,1.0,1.25])
    
    # ax.set_zlim(0,1.25)
    # ax.view_init(elev=22,azim=-170)
    # print()



    ## IMPACT ANGLE SUCCESS RATE PLOT
    # plot_raw(Z_data='impact_eul_mean',XY_data=("impact_vx_mean","impact_vz_mean"),XY_str=("impact_vx_mean","impact_vz_mean"),color_data='landing_rate_4_leg',polar=True)
    # plot_plotly(df_raw,Z_data='impact_eul_mean',XY_data=("impact_vx_mean","impact_vz_mean"),XY_str=("impact_vx_mean","impact_vz_mean"),color_data='landing_rate_4_leg',polar=True)


    # plt.show()

    