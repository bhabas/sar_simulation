## IMPORT MODULES
import pandas as pd
import numpy as np
from sklearn import linear_model
from scipy.interpolate import griddata
import scipy.odr
import plotly.graph_objects as go
import os,sys

## IMPORT PLOTTING MODULES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm

sys.path.insert(0,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_data/')
from data_analysis.Data_Analysis import DataFile

os.system("clear")

## RAW DATA FUNCTIONS
def plot_raw(df,Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
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
    ax.scatter(X,Y,Z,c=C,cmap=cmap,norm=norm,alpha=0.5,depthshade=False)
    fig.colorbar(mpl.cm.ScalarMappable(cmap=cmap,norm=norm),label=color_str)


    # PLOT LIMITS AND INFO
    ax.set_zlabel(Z_data)
    # ax.set_ylim(2,7)
    fig.tight_layout()

    return fig,ax

def plot_plotly(df,Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
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


## OPTIMAL DATA FUNCTIONS
def plot_best(Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if XY_data==None:

        if polar==True:
            X = df_max['vx']
            Y = df_max['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df_max['phi_IC']
            Y = df_max['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df_max[XY_data[0]]
        Y = df_max[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

        
    Z = df_max[Z_data]
    C = df_max[color_data]

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
        X = df_max['vx']
        Y = df_max['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df_max['phi_IC']
        Y = df_max['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df_max[Z_data]
    C = df_max[color_data]

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
        X = df_max['vx']
        Y = df_max['vz']

        ax.set_xlabel('Vel_x (m/s)')
        ax.set_ylabel('Vel_z (m/s)')

    else:
        X = df_max['phi_IC']
        Y = df_max['vel_IC']

        ax.set_xlabel('phi (deg)')
        ax.set_ylabel('Vel (m/s)')

        
    Z = df_max[Z_data]
    C = df_max[color_data]

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

def plot_polar_smoothed():

    # COLLECT DATA
    R = df_max.iloc[:]['vel_IC']
    Theta = df_max.iloc[:]['phi_IC']
    Z = df_max.iloc[:]['My_d']

    # SOMETHING ABOUT DEFINING A GRID
    ri = np.linspace(R.min(),R.max(),(len(Z)//15))
    thetai = np.linspace(Theta.min(),Theta.max(),(len(Z)//15))
    zi = griddata((R, Theta), Z, (ri[None,:], thetai[:,None]), method='linear')
    r_ig, theta_ig = np.meshgrid(ri, thetai)
    

    ## INIT PLOT INFO
    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=0,vmax=10)
    
    ax.contourf(np.radians(theta_ig),r_ig,zi,cmap=cmap,norm=norm,levels=20)
    ax.set_thetamin(25)
    ax.set_thetamax(90)
    ax.set_rmin(0)
    ax.set_rmax(4)
    


    ## AXIS LABELS    
    ax.text(np.radians(10),2,'Flight Velocity (m/s)',
        rotation=25,ha='center',va='center')

    ax.text(np.radians(60),4.5,'Flight Angle (deg)',
        rotation=0,ha='left',va='center')

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
    model_config = "Narrow-Long"
    df_raw = pd.read_csv(f"Projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_raw = df_raw.query(f"landing_rate_4_leg >= {0.0}")

    


    ## MAX LANDING RATE DATAFRAME
    idx = df_raw.groupby(['vel_IC','phi_IC'])['landing_rate_4_leg'].transform(max) == df_raw['landing_rate_4_leg']
    df_max = df_raw[idx].reset_index()


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


    ## LANDING RATE POLAR PLOT
    # plot_polar_smoothed()
    plt.show()

    