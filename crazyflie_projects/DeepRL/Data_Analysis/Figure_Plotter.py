import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)


## LIST OF COLLECTED REWARD DATA
filelist = [
    "DeepRL_Training_SAC-08_30_RewardData.csv",
    "DeepRL_Training_SAC-08_31_RewardData.csv",
    "DeepRL_Training_SAC-09_01_RewardData.csv",
    "DeepRL_Training_SAC-09_02_RewardData.csv",
    "DeepRL_Training_SAC-09_06_RewardData.csv"
]


def Reward_Plot(saveFig=False):

    ## CREATE FIGURE
    fig = plt.figure(figsize=(6, 3))
    ax = fig.add_subplot(111)

    ## PRE-ALLOCATE ARRAY FOR REWARD DATA
    Reward_vals = np.zeros((500,1))

    ## INTERPOLATE REWARD DATA ONTO SAME EPISODE SCALE AND PLOT DATA
    for ii,file in enumerate(filelist):
        df = pd.read_csv(f"{BASE_PATH}/crazyflie_projects/DeepRL/Data_Logs/{file}")

        x = df["Episode"].to_numpy()
        y = df["Reward"].to_numpy()
        f = interp1d(x,y)
        x_new = np.linspace(5,6000,num=500,endpoint=True)
        y_new = f(x_new).reshape(-1,1)
        Reward_vals = np.hstack((Reward_vals,y_new))

        ax.plot(x_new,y_new,alpha=0.25)

    ## AVERAGE REWARD DATA FROM TRIALS AND PLOT
    Reward_vals = Reward_vals[:,1:]
    Reward_mean = np.mean(Reward_vals,axis=1)
    ax.plot(x_new,Reward_mean,color="tab:orange",label="Average Reward")

    ## PLOT CONFIGURATION
    ax.set_xlabel("Episode")
    ax.set_ylabel("Reward")
    ax.legend(loc="lower right")
    ax.set_title("Reward vs Training Episode")

    ax.set_ylim(0,1)
    ax.grid()
    plt.tight_layout()

    if saveFig==True:
        plt.savefig(f'NL_DeepRL_Reward_Plot.pdf',dpi=300)

    plt.show()

if __name__ == '__main__':
    Reward_Plot(saveFig=True)