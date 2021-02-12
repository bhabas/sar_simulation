from sklearn.linear_model import LinearRegression
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm
from scipy.interpolate import griddata


# TEMPLATE FOR LINEAR REGRESSION
from sklearn.linear_model import LinearRegression

# FUNCTION FOR PLOTTING 3D DATA 
def plot_3d_data(df,x1,x2,y,x1label="",x2label="",ylabel="",title="",xlim=None,ylim=None,zlim=None):
    # Input:   df -> [df] data frame
    #          x1 -> [string] dependant variable 1 df name
    #          x2 -> [string] dependant variable 1 df name
    #          y  -> [string] independant variable df name
    #          x1label -> [string] dependant variable 1 plot name
    #          x2label -> [string] dependant variable 1 plot name
    #          ylabel  -> [string] independant variable plot name
    #          title   -> [string] plot title 
    
    # Output   ax -> [handle] 3d plot handle
    
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")
    ax.scatter(df[x1],df[x2],df[y])

    ax.set_xlabel(x1label)
    ax.set_ylabel(x2label)
    ax.set_zlabel(ylabel)
    ax.set_title(title)

    if xlim: ax.set_xlim(xlim[0],xlim[1])
    if ylim: ax.set_ylim(ylim[0],ylim[1])
    if zlim: ax.set_zlim(zlim[0],zlim[1])
        
    plt.show()
    
    return ax


# function to perfrom both multivariate and univarate regression
def linreg(df,x1,x2,y):
    # function to perfrom both multivariate and univarate regression
    
    # input: 1) df['x1'] -> no need to do anything special, just make sure there are no NaN values
    #        2) df['x2']
    #        3) df['y']
    
    # Output: 1) x1 array -> formatted for plotting (1d np arrary)
    #         2) x2 array
    #         3) y array
    #         4) multivariate results in tuple
    #         5) univariate (x1) results in tuple
    #         6) univariate (x2)results in tuple
    #            -> tuple indices are: 0) ypred (ready for plotting plane or line)
    #                                  1) intercepts
    #                                  2) coefficients
    #                                  3) R2 
    x1,x2 = df[x1],df[x2]

    if y == "impact_eul": y = -df[y]
    else: y = df[y]

    
    length = len(x1)
    x1,x2,Y = x1.values.reshape(length,1),x2.values.reshape(length,1),y.values.reshape(length,1)
    
    # start with multivaraite
    X = np.concatenate((x1,x2),axis = 1)
    reg = LinearRegression(normalize=True)
    reg.fit(X,Y)

    intercept = reg.intercept_ 
    coef = reg.coef_
    
    R2 = reg.score(X,Y) # built in function for r^2
    
    ypred = reg.predict(X) # plot x vs. ypred for line or plane
    
    mv_reg = (ypred.reshape(length,), intercept,coef,R2)
    
    # then do x1 only
    reg = LinearRegression(normalize=True)
    reg.fit(x1,Y)

    intercept = reg.intercept_ 
    coef = reg.coef_
    
    R2 = reg.score(x1,Y) # built in function for r^2
    
    ypred = reg.predict(x1) # plot x vs. ypred for line or plane
    
    x1_reg = (ypred.reshape(length,), intercept,coef,R2)
    
    # then do x2 only
    reg = LinearRegression(normalize=True)
    reg.fit(x2,Y)

    intercept = reg.intercept_ 
    coef = reg.coef_
    
    R2 = reg.score(x2,Y) # built in function for r^2
    
    ypred = reg.predict(x2) # plot x vs. ypred for line or plane
    
    x2_reg = (ypred.reshape(length,), intercept,coef,R2)
    
    x1val = x1.reshape(length,)
    x2val = x2.reshape(length,)
    Yval = Y.reshape(length,)
    return x1val, x2val, Yval, mv_reg, x1_reg, x2_reg



# FUNCTION TO ADD 3D REGRESSION PLANE AND PRINT EQNS/R2
def reg_3d(ax,x1,x2,mv,x1name,x2name,yname):
    # function to generate and plot regression data
    
    # Input: 1) ax: handle for 3d plot
    #        2) tmp_df: data frame with nan rows removed
    #        3) x1name: string with name for x1 variable
    #        4) x2name: string with name for x2 variable
    #        5) yname: string with name for y variable

    # perform uni/multi variate regression
    #x1,x2,y,mv,var1,var2 = linreg2(tmp_df[x1name],tmp_df[x2name],tmp_df[yname])
    
    # print r2 values
    #print(x1name+" and " + x2name + " vs. " + yname+ " r2 = ", mv[3])
    
    print("------3D Regression Results------ \n")
    print("r2 = ", round(mv[3],4))
    print("Equation: " , yname , " = " , round(mv[1][0],6) , " + " , round(mv[2][0][0],6) , "*" , x1name, " + " , round(mv[2][0][1],6) , "*" , x2name)
    
    # add plane to prevoius 3d polot
    ax.plot_trisurf(x1.T,x2.T,mv[0].T)

    plt.show()

# FUNCTION FOR GENERATING 2D REGRESSION PLOTS AND LINE EQN
def reg_2d(x1val,x2val,yval,x1_reg,x2_reg,x1name,x2name,yname):
    # function to generate and plot regression data
    
    # Input: 1) ax: handle for 3d plot
    #        2) tmp_df: data frame with nan rows removed
    #        3) x1name: string with name for x1 variable
    #        4) x2name: string with name for x2 variable
    #        5) yname: string with name for y variable

    
    print("------2D Regression Results------ \n")
    
    print(x1name + " results:")
    print("r2 = ", round(x1_reg[3],4))

    
    print("Equation: " , yname , " = " , round(x1_reg[1][0],6) , " + " , round(x1_reg[2][0][0],6) , "*" , x1name)
    
    print("\n")
    
    print(x2name + " results:")
    print("r2 = ", round(x2_reg[3],4))

    
    print("Equation: " , yname , " = " , round(x2_reg[1][0],6) , " + " , round(x2_reg[2][0][0],6) , "*" , x2name)
    
    # generate plots for 2d regression
    fig = plt.figure()
    ax = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)


    # plot lin reg for x1 vs y
    ax.scatter(x1val,yval)
    ax.plot(x1val,x1_reg[0],'r')
    ax.set_xlabel(x1name)
    ax.set_ylabel(yname)
    # plot lin reg for x2 vs y
    ax2.scatter(x2val,yval)
    ax2.plot(x2val,x2_reg[0],'r')
    ax2.set_xlabel(x2name)

    plt.show()

# COLOR SURFACE PLOT
def color_plot(df,x1,x2,y,x1l,x2l,yl,title,clim,clabel,xlim=None,ylim=None,zlim=None):
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111,projection="3d")
    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=clim[0],vmax=clim[1])
    pcm = ax.plot_trisurf(df[x1],df[x2],df[y,'mean'],cmap = cmap,norm=norm)
    fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label=clabel)
    df
    if xlim: ax.set_xlim(xlim[0],xlim[1])
    if ylim: ax.set_ylim(ylim[0],ylim[1])
    if zlim: ax.set_zlim(zlim[0],zlim[1])
    
    ax.set_xlabel(x1l)
    ax.set_ylabel(x2l)
    ax.set_zlabel(yl)
    ax.set_title(title)


    plt.show()

    fig = plt.figure()


    ax = fig.add_subplot(111,projection="3d")



    X = df[x1]
    Y = df[x2]
    Z = df[y,'mean']

    xi = np.linspace(X.min(),X.max(),(len(Z)//3))
    yi = np.linspace(Y.min(),Y.max(),(len(Z)//3))
    zi = griddata((X, Y), Z, (xi[None,:], yi[:,None]), method='linear')
    xig, yig = np.meshgrid(xi, yi)

    cmap = mpl.cm.jet
    norm = mpl.colors.Normalize(vmin=clim[0],vmax=clim[1])


    surf = ax.plot_surface(xig, yig, zi,cmap=cmap,norm=norm)
    # surf = ax.contour(xig, yig, zi,cmap=cmap,norm=norm)

    fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label=clabel)

    if xlim: ax.set_xlim(xlim[0],xlim[1])
    if ylim: ax.set_ylim(ylim[0],ylim[1])
    if zlim: ax.set_zlim(zlim[0],zlim[1])

    ax.set_xlabel(x1l)
    ax.set_ylabel(x2l)
    ax.set_zlabel(yl)
    ax.set_title(title)

    plt.show()