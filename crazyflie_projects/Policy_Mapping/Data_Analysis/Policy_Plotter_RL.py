import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import plotly.graph_objects as go


BASEPATH = "crazyflie_projects/Policy_Mapping/Data_Logs/"

df = pd.read_csv(f"{BASEPATH}/test_file_1.csv")

all_files = [
    f"{BASEPATH}/test_file_1.csv",
    f"{BASEPATH}/test_file_2.csv",
    f"{BASEPATH}/test_file_3.csv",
    f"{BASEPATH}/test_file_4.csv",
    f"{BASEPATH}/test_file_5.csv",
    f"{BASEPATH}/test_file_6.csv",
    f"{BASEPATH}/test_file_7.csv"
]

# all_files = [
#     f"{BASEPATH}/test_file_1.csv",
#     f"{BASEPATH}/test_file_2.csv"
# ]

df = pd.concat((pd.read_csv(f) for f in all_files))
df = df.sample(frac=0.01)
df = df.query('success_flag == True')





X_1 = df['OF_y']
Y_1 = 1/df['Tau']
Z_1 = df['d_ceil']
C_1 = df['My_d']




fig = go.Figure(data=[go.Scatter3d(x=X_1, y=Y_1, z=Z_1,
                                   mode='markers',marker=dict(
        size=2,
        color=C_1.astype(float),                # set color to an array/list of desired values
        colorscale='Jet',   # choose a colorscale
        opacity=0.7
    ))])
    
# CREATE PLOTS AND COLOR BAR
fig.update_layout(
    scene = dict(
        xaxis = dict(nticks=4, range=[-15,0],),
        yaxis = dict(nticks=4, range=[0,7],),
        zaxis = dict(nticks=4, range=[0,1],)
    ),
)


fig.show()
