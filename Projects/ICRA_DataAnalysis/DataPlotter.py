## IMPORT MODULES
import pandas as pd
import numpy as np
from sklearn import linear_model
from scipy.interpolate import griddata
import scipy.odr
import plotly.graph_objects as go
import os

## IMPORT PLOTTING MODULES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm

os.system("clear")








# ======================================
##        LANDING RATE DATA (RAW)
# ======================================

def plot_raw(Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if XY_data==None:
        if polar==True:
            X = df_raw['vx']
            Y = df_raw['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df_raw['phi_IC']
            Y = df_raw['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df_raw[XY_data[0]]
        # vz = df_raw['flip_vz_mean']
        # d = 2.1 - df_raw['flip_height_mean']
        # X = vz**2/d**2
        Y = df_raw[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

        
    Z = df_raw[Z_data]
    C = df_raw[color_data]

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
    ax.set_ylim(2,7)
    ax.set_title("Raw Data")
    fig.tight_layout()

def plot_raw_plotly(Z_data:str,color_data:str='landing_rate_4_leg',color_str=None,color_norm=None,polar=True,XY_data=None,XY_str=None):

    
    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    ## DEFINE PLOT VARIABLES
    if XY_data==None:
        if polar==True:
            X = df_raw['vx']
            Y = df_raw['vz']

            ax.set_xlabel('Vel_x (m/s)')
            ax.set_ylabel('Vel_z (m/s)')

        else:
            X = df_raw['phi_IC']
            Y = df_raw['vel_IC']

            ax.set_xlabel('phi (deg)')
            ax.set_ylabel('Vel (m/s)')
    else:
        X = df_raw[XY_data[0]]
        # vz = df_raw['flip_vz_mean']
        # d = 2.1 - df_raw['flip_height_mean']
        # X = vz**2/d**2
        Y = df_raw[XY_data[1]]

        ax.set_xlabel(XY_str[0])
        ax.set_ylabel(XY_str[1])

        
    Z = df_raw[Z_data]
    C = df_raw[color_data]
   

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
        xaxis_title = "OF_y",
        yaxis_title = "RREV",
        zaxis_title = f"Raw Data",
        # xaxis = dict(nticks=4, range=[0,5]),
        yaxis = dict(range=[0,8]),
        # zaxis = dict(nticks=4, range=[0,360])
        ))

    fig.show()



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


def plot_all_eul():

    ## INITIALIZE FIGURE
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")

    X = df_raw['impact_vx_arr']
    for idx, i in enumerate(X): 
            X[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    X = np.concatenate(X,axis=0)

    Y = df_raw['impact_vz_arr']
    for idx, i in enumerate(Y): 
            Y[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    Y = np.concatenate(Y,axis=0)

    Z = df_raw['impact_eul_arr']
    for idx, i in enumerate(Z): 
            Z[idx] = np.fromstring(i[1:-1], dtype=float, sep=' ')
    Z = np.concatenate(Z,axis=0)

    C = df_raw['contact_arr']
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

def plot_polar(Z_data:str):
    azimuths = np.radians(np.linspace(0, 90, 20))
    zeniths = np.arange(0, 70, 10)

    r, theta = np.meshgrid(zeniths, azimuths)
    values = np.random.random((azimuths.size, zeniths.size))

    #-- Plot... ------------------------------------------------
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    ax.contourf(theta, r, values)

    ax.set_thetamin(0)
    ax.set_thetamax(90)

    plt.show()


if __name__ == '__main__':

    ## FULL DATAFRAME
    model_config = "ExtraNarrow-Long"
    df_raw = pd.read_csv(f"Projects/ICRA_DataAnalysis/{model_config}_2-Policy/{model_config}_2-Policy_Summary.csv")
    df_raw = df_raw.query(f"landing_rate_4_leg >= {0.0}")


    ## MAX LANDING RATE DATAFRAME
    idx = df_raw.groupby(['vel_IC','phi_IC'])['landing_rate_4_leg'].transform(max) == df_raw['landing_rate_4_leg']
    df_max = df_raw[idx].reset_index()



    # plot_best_surface_smoothed('landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)
    # plot_raw(Z_data='My_d',XY_data=("flip_height_mean","impact_eul_mean"),XY_str=("h","eul impact"),color_data='landing_rate_4_leg',polar=True)
    # plot_raw(Z_data='impact_eul_mean',XY_data=("impact_vx_mean","impact_vz_mean"),XY_str=("impact_vx_mean","impact_vz_mean"),color_data='impact_eul_std',polar=True)
    # plot_best(dataName='impact_eul_mean',color_data='landing_rate_4_leg',polar=True)
    # plot_best_surface(dataName='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)
    # plot_best_surface_smoothed(dataName='RREV_threshold',color_data='RREV_threshold',polar=False)


    ## NON-SMOOTH LANDING RATE SURFACE
    # plot_best_surface(Z_data='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)

    ## SMOOTH LANDING RATE SURFACE
    # plot_best_surface_smoothed(Z_data='landing_rate_4_leg',color_data='landing_rate_4_leg',polar=True)

    ## RREV VS OF_d VS My_d
    # plot_raw(Z_data='My_d',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg')
    
    # plot_raw_plotly(Z_data='My_d',XY_data=('OF_y_flip_mean','RREV_flip_mean'),XY_str=('OF_y','RREV'),color_data='landing_rate_4_leg')


    ## 
    # plot_raw(Z_data='impact_eul_mean',XY_data=("impact_vx_mean","impact_vz_mean"),XY_str=("impact_vx_mean","impact_vz_mean"),color_data='landing_rate_4_leg',polar=True)

    # plot_polar('test')

    r = df_max.iloc[:]['vel_IC']
    theta = df_max.iloc[:]['phi_IC']
    colors = df_max.iloc[:]['landing_rate_4_leg']
    

    # r, theta = np.meshgrid(zeniths, azimuths)
    # values = np.random.random((azimuths.size, zeniths.size))

    #-- Plot... ------------------------------------------------
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    # ax.contourf(theta, r, values)
    cmap = mpl.cm.rainbow
    norm = mpl.colors.Normalize(vmin=0,vmax=1)
    ax.scatter(np.radians(theta),r,c=colors,cmap=cmap,norm=norm)

    ax.set_thetamin(0)
    ax.set_thetamax(90)
    ax.set_rmin(0)
    ax.set_rmax(5)
    plt.show()