# To add a new cell, type '# %%'
# To add a new markdown cell, type '# %% [markdown]'
# %%
from IPython import get_ipython

# %% [markdown]
# # Module Imports

# %%
## IMPORT MODULES
import pandas as pd
import numpy as np
from sklearn import linear_model


# %%
## IMPORT PLOTTING MODULES
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib as mpl
from matplotlib import cm
# get_ipython().run_line_magic('matplotlib', 'widget')

# %% [markdown]
# # Dataframe

# %%
## FULL DATAFRAME
df_raw = pd.read_csv("NS_2-Policy_Summary.csv")

## Drop extraneous data points
df_raw = df_raw.drop(df_raw[
    (df_raw['landing_rate']<=0.1) & 
    (df_raw['vz_d']>=2.75)
    ].index)

df_raw = df_raw.drop(df_raw[
    (df_raw['landing_rate']<=0.1) & 
    (df_raw['vx_d']>=1.0)
    ].index)

df_raw_avg = df_raw.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()


# df_temp = df.query(f"landing_rate>={0.2}")
# df_temp_avg = df_raw.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()


df = df_raw
df['My_d'] = df['My_d'].apply(lambda x: np.abs(x)) # Convert My to magnitude
# df['My_d'] = df['My_d'].apply(lambda x: 7.7 if x>7.7 else x) # Cap My_d to max moment
df = df.drop(df[(df['vz_d']<= 2.25) & (df['vx_d']<= 0.75)].index) # Drop corner with no successful landings
df = df.dropna()

## AVERAGED DATAFRAME
df_avg = df.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()


# %%
df

# %% [markdown]
# # Landing Rate Data

# %%
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")
ax.scatter(df['vx_d'],df['vz_d'],df['landing_rate'])

ax.set_zlim(0,1)

ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('Landing Rate')
ax.set_title('Landing Rate (Raw Data)')

plt.show()


# %%
## AVG LANDING RATE SURFACE
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")
cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=1)
pcm = ax.plot_trisurf(df_raw_avg['vx_d'],df_raw_avg['vz_d'],df_raw_avg['landing_rate','mean'],cmap = cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label="Landing %")
df
ax.set_zlim(0,1)

ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('Landing Rate')
ax.set_title('Avg Landing Rate for Final 3 Episodes')


plt.show()

# %% [markdown]
# # RREV vs IC

# %%
## RREV Threshold vs IC
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

ax.scatter(df['vx_d'],df['vz_d'],df['RREV_threshold'])


ax.set_xlim(0,2.75)
ax.set_ylim(1.0,4.5)


ax.set_xlabel('Vx_d')
ax.set_ylabel('Vz_d')
ax.set_zlabel('RREV_threshold')
ax.set_title('RREV_thr vs IC - (Raw Data)')


plt.show()


# %%

## Avg RREV_thr vs IC
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=4,vmax=5)
pcm = ax.plot_trisurf(df_avg['vx_d'],df_avg['vz_d'],df_avg['RREV_threshold','mean'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label="RREV_threshold")

ax.set_xlim(0,2.75)
ax.set_ylim(1.0,4.5)
ax.set_zlim(3,6)

ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('RREV_threshold')

ax.set_title('Avg RREV_thr vs IC')
plt.show()

# %% [markdown]
# # Rotation Time Data

# %%
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

df_temp = df.query(f"landing_rate>={0.1}")

cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=4,vmax=5)
pcm = ax.scatter(df_temp['vx_d'],df_temp['vz_d'],df_temp['impact_tdelta'])
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label="RREV_threshold")


ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('Delta_t [s]')

ax.set_title('Time Rotating vs IC (Raw Data)')

ax.set_zlim(0,0.5)




plt.show()


# %%
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

df_temp = df.query(f"landing_rate>={0.1}")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=4)

pcm = ax.scatter(df_temp['RREV_trigger'],df_temp['landing_rate'],df_temp['impact_tdelta'],c=df_temp['vz_d'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Vz_d')


ax.set_xlabel('RREV_trigger')
ax.set_ylabel('Landing_rate')
ax.set_zlabel('Delta_t [s]')

ax.set_title('Time Rotating vs RREV (Raw Data)')

ax.set_zlim(0,0.5)




plt.show()

# %% [markdown]
# # My_d vs IC Data

# %%
## Define Dataframe

df_temp = df.query(f"landing_rate>={0.1}")


fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

pcm = ax.scatter(df['vx_d'],df['vz_d'],df['My_d'])



ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('My_d [N*mm]')

ax.set_title('My_d vs IC (Raw Data)')

## I have more data for vz = (1.5,2.75) than vz = (3.00,4.0) so My_d differences are likely not a trend, just sparse data




plt.show()

# %% [markdown]
# # Impact Angle Data

# %%
df_temp = df.query(f"impact_eul<={-60}")
# df_temp_avg = df_raw.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()



## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)
pcm = ax.scatter(df_temp['vx_d'],df_temp['vz_d'],-df_temp['impact_eul'],c=df_temp['landing_rate'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing %')

ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('Impact Angle')


ax.set_zlim(60,180)
ax.set_zticks([60,90,120,150,180])

## Failed landings in corner
plt.show()


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"impact_eul<={-60}")
df_temp_avg = df_temp.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()


## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=180)
ax.plot_trisurf(df_temp_avg['vx_d'],df_temp_avg['vz_d'],-df_temp_avg['impact_eul','mean'],cmap=cmap)


ax.set_xlim(0,2.75)
ax.set_ylim(1.0,4.0)
ax.set_zlim(60,180)
ax.set_zticks([60,90,120,150,180])


ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('Impact Angle')

ax.set_title('Avg Impact Angle vs IC')
plt.show()


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"impact_eul<={-60}")
df_temp_avg = df_temp.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()

import matplotlib.tri as tri

triang = tri.Triangulation(df_temp_avg['vx_d'],df_temp_avg['vz_d'])


cond1 = np.less(df_temp_avg['vx_d'],1.0)
cond2 = np.less(df_temp_avg['vz_d'],2.0)
mask = np.where(cond1 & cond2,0,1)
triang.set_mask(mask)





## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=180)
ax.plot_trisurf(triang,-df_temp_avg['impact_eul','mean'],cmap=cmap)







ax.set_xlim(0,2.75)
ax.set_ylim(1.0,4.0)
ax.set_zlim(60,180)
ax.set_zticks([60,90,120,150,180])


ax.set_xlabel('vx_d')
ax.set_ylabel('vz_d')
ax.set_zlabel('Impact Angle')

ax.set_title('Avg Impact Angle vs IC')
plt.show()

# %% [markdown]
# # Moment vs RREV

# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}")
df_temp_avg = df_temp.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()


## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111)


cmap = mpl.cm.Greys
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.scatter(df_temp['RREV_threshold'],df_temp['My_d'],c=df_temp['landing_rate'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing %')

ax.grid()
ax.set_xlim(2,7)
ax.set_ylim(0,10)
ax.hlines(7.77,2,7)
ax.text(2.05,8.0,'Motors Maxed Out')

ax.set_xlabel('RREV_thr')
ax.set_ylabel('My_d [N*mm]')
ax.set_title('Policy: My_d vs RREV_thr')

plt.show()


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}")
df_temp_avg = df_temp.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()

## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111)


cmap = mpl.cm.jet
norm = mpl.colors.Normalize(vmin=0,vmax=3)

ax.scatter(df_temp['RREV_threshold'],df_temp['My_d'],c=df_temp['vz_d'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Vz_d')

ax.grid()
ax.set_xlim(2,7)
ax.set_ylim(0,10)
ax.hlines(7.77,2,7)
ax.text(2.05,8.0,'Motors Maxed Out')

ax.set_xlabel('RREV_thr')
ax.set_ylabel('My_d [N*mm]')
ax.set_title('Policy: My_d vs RREV_thr')
## As Vz increases RREV_trigger decreases to preserve time to contact. It also finds a host of My_d that work and they vary because of the robustness of the legs and the fact that landing success isn't deterministic.
plt.show()


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}")
df_temp_avg = df_temp.groupby(['vz_d','vx_d']).agg([np.mean,np.std]).reset_index()

## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.scatter(df_temp['RREV_threshold'],df_temp['vx_d'],df_temp['My_d'],c=df_temp['landing_rate'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing %')

ax.set_zlim(0,10)
ax.set_xlim(2,7)

ax.set_xlabel('RREV_trigger')
ax.set_ylabel('vx_d')
ax.set_zlabel('My_d')


plt.show()


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}")

## AVG LANDING RATE PLOT
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.scatter(df_temp['RREV_threshold'],df_temp['landing_rate'],df_temp['My_d'],c=df_temp['landing_rate'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing %')

ax.set_zlim(0,10)
ax.set_xlim(3,7)

ax.set_xlabel('RREV_trigger')
ax.set_ylabel('Landing_Rate')
ax.set_zlabel('My_d')


plt.show()


# %%
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)

pcm = ax.scatter(df['vx_d'],df['My_d'],df['RREV_threshold'],c=df['landing_rate'],cmap=cmap)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing %')

# ax.set_xlim(3,7)
# ax.set_ylim(0,-10)
# ax.set_zlim(0,-10)

ax.set_xlabel('Vx_d')
ax.set_ylabel('OF_y_trigger')
ax.set_zlabel('RREV_threshold')


plt.show()

# %% [markdown]
# # POLICY FITTING
# %% [markdown]
# ## RAW POLICY RELATION

# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max

## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.scatter(df_temp['RREV_threshold'],df_temp['OF_y'],df_temp['My_d'],c=df_temp['landing_rate'],cmap=cmap,norm=norm)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)

ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')

ax.set_title('Raw Policy Relation')


plt.show()

# Note: the planar look to the data


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max
df_temp = df_temp.query(f"RREV_threshold>{3.5}") # Remove outlier data
df_temp = df_temp.query(f"landing_rate>={0.8}")

## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.scatter(df_temp['RREV_threshold'],df_temp['OF_y'],df_temp['My_d'],c=df_temp['landing_rate'],cmap=cmap,norm=norm)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)

ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')

ax.set_title('Raw Policy Relation (LR > 80%)')


plt.show()

# Note: the planar look to the data


# %%
## CLEAN DATAFRAME
df_temp = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max
df_temp = df_temp.query(f"RREV_threshold>{3.5}") # Remove outlier data
df_temp = df_temp.query(f"landing_rate>={0.5}")

## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")

cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)

ax.scatter(df_temp['RREV_threshold'],df_temp['OF_y'],df_temp['My_d'],c=df_temp['landing_rate'],cmap=cmap,norm=norm)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)

ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')

ax.set_title('Raw Policy Relation (LR > 50%)')


plt.show()

# Note: the planar look to the data

# %% [markdown]
# # LINEAR MODEL FIT
# %% [markdown]
# ## Regression with y-based loss

# %%
## CLEAN DATAFRAME
df_clean = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max
df_clean = df_clean.query(f"RREV_threshold>{3.5}") # Remove outlier data
df_reg1 = df_clean.query(f"landing_rate>={0.6}")


X = df_reg1[['RREV_threshold','OF_y']]
Y = df_reg1['My_d']

reg = linear_model.LinearRegression(normalize=True)
reg.fit(X,Y)
intercept = reg.intercept_
coeffs = reg.coef_

print(f"Equation: My_d = {intercept:.2f} + {coeffs[0]:.2f}*RREV + {coeffs[1]:.2f}*OF_y \n")


## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")


My_reg = intercept + coeffs[0]*df_reg1['RREV_threshold'] + coeffs[1]*df_reg1['OF_y']
ax.plot_trisurf(df_reg1['RREV_threshold'],df_reg1['OF_y'],My_reg,label='Linear_Regression',color='blue',zorder=2)


cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.scatter(df_reg1['RREV_threshold'],df_reg1['OF_y'],df_reg1['My_d'],c=df_reg1['landing_rate'],cmap=cmap,norm=norm,zorder=1)



ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)


ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')


plt.show()

## Regression Analysis

import statsmodels.api as ssm       # for detail description of linear coefficients, intercepts, deviations, and many more
X=ssm.add_constant(X)               # to add constant value in the model
model= ssm.OLS(Y,X).fit()           # fitting the model
predictions= model.summary()        # summary of the model
predictions




# %%
# def r_squared(y, y_hat): #Manual R^2 value
#     y_bar = y.mean()
#     ss_tot = ((y-y_bar)**2).sum()
#     ss_res = ((y-y_hat)**2).sum()
#     return 1 - (ss_res/ss_tot)

# R2 = r_squared(df_reg['My_d'],My_reg)
# print(f"R^2: {R2:.3f}")

# %% [markdown]
# ## Regression with x-based loss
# 

# %%
## CLEAN DATAFRAME
df_clean = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max
df_clean = df_clean.query(f"RREV_threshold>{3.5}") # Remove outlier data
df_reg1 = df_clean.query(f"landing_rate>={0.6}")


X = df_reg1[['My_d','OF_y']]
Y = df_reg1['RREV_threshold']

reg = linear_model.LinearRegression(normalize=True)
reg.fit(X,Y)
intercept = reg.intercept_
coeffs = reg.coef_

print(f"Equation: RREV_reg = {intercept:.2f} + {coeffs[0]:.2f}*My_d + {coeffs[1]:.2f}*OF_y \n")


## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")


RREV_reg = intercept + coeffs[0]*df_reg1['My_d'] + coeffs[1]*df_reg1['OF_y']
ax.plot_trisurf(RREV_reg,df_reg1['OF_y'],df_reg1['My_d'],label='Linear_Regression',color='blue',zorder=2)


cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=5)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.scatter(df_reg1['RREV_threshold'],df_reg1['OF_y'],df_reg1['My_d'],c=df_reg1['vz_d'],cmap=cmap,norm=norm,zorder=1)



ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)


ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')


plt.show()

## Regression Analysis

import statsmodels.api as ssm       # for detail description of linear coefficients, intercepts, deviations, and many more
X=ssm.add_constant(X)               # to add constant value in the model
model= ssm.OLS(Y,X).fit()           # fitting the model
predictions= model.summary()        # summary of the model
predictions




# %%



# %%
## PLOT FULL DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")


ax.plot_trisurf(RREV_reg,df_reg1['OF_y'],df_reg1['My_d'],label='Linear_Regression',color='blue',zorder=2)


cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.scatter(df_clean['RREV_threshold'],df_clean['OF_y'],df_clean['My_d'],c=df_clean['landing_rate'],cmap=cmap,norm=norm,zorder=1)



ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)


ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')


plt.show()


# %%
## CLEAN DATAFRAME
df_clean = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max
df_clean = df_clean.query(f"RREV_threshold>{3.5}") # Remove outlier data
df_reg1 = df_clean.query(f"landing_rate>={0.8}")


fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")


for ii in np.arange(1,0.2,-.2):
    df_reg1 = df.query(f"landing_rate<={ii} & landing_rate>{ii-.2} & RREV_threshold>{3.5}")
    
    X = df_reg1[['My_d','OF_y']]
    Y = df_reg1['RREV_threshold']

    reg2 = linear_model.LinearRegression(normalize=True)
    reg2.fit(X,Y)

    intercept = reg2.intercept_
    coeffs = reg2.coef_

    # print(intercept)
    # print(coeffs)

    RREV_reg = intercept + coeffs[0]*df_reg1['My_d'] + coeffs[1]*df_reg1['OF_y']
    ax.plot_trisurf(RREV_reg,df_reg1['OF_y'],df_reg1['My_d'],label='Linear_Regression')

    R2 = r_squared(df_reg1['RREV_threshold'],RREV_reg)
    print(f"R^2: {R2:.3f}")

ax.set_xlim(6,3.5)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)


ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')


plt.show()


# %%



# %%
import scipy.odr

## CLEAN DATAFRAME
df_clean = df.query(f"My_d<={7.7}") # Remove extraneous moments higher than max
df_clean = df_clean.query(f"RREV_threshold>{3.5}") # Remove outlier data
df_reg1 = df_clean.query(f"landing_rate>={0.8}")


# X = df_reg1[['My_d','OF_y']]
x1 = df_reg1['RREV_threshold'].to_numpy()
x2 = df_reg1['OF_y'].to_numpy()
X = np.row_stack( (x1, x2) )

Y = df_reg1['My_d'].to_numpy()

def linfit(beta,x):
    return beta[0]*x[0] + beta[1]*x[1] + beta[2]

linmod = scipy.odr.Model(linfit)
data = scipy.odr.Data(X,Y)
odrfit = scipy.odr.ODR(data,linmod,beta0 = [-0.1,0.1,4.0])
odrres = odrfit.run()
odrres.pprint()




## PLOT DATA
fig = plt.figure()
ax = fig.add_subplot(111,projection="3d")


My_reg = odrres.beta[0]*df_reg1['RREV_threshold'] + odrres.beta[1]*df_reg1['OF_y'] + odrres.beta[2]
ax.plot_trisurf(df_reg1['RREV_threshold'],df_reg1['OF_y'],My_reg,label='Linear_Regression',color='blue',zorder=2)


cmap = mpl.cm.rainbow
norm = mpl.colors.Normalize(vmin=0,vmax=1)
fig.colorbar(mpl.cm.ScalarMappable(norm=norm, cmap=cmap),label='Landing Rate')

ax.scatter(df_reg1['RREV_threshold'],df_reg1['OF_y'],df_reg1['My_d'],c=df_reg1['landing_rate'],cmap=cmap,norm=norm,zorder=1)



ax.set_xlim(6,2)
ax.set_ylim(-10,0)
ax.set_zlim(0,10)


ax.set_xlabel('RREV')
ax.set_ylabel('OF_y')
ax.set_zlabel('My_d')


plt.show()


# %%



# %%



