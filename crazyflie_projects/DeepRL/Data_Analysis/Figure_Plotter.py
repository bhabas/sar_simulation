import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)


def Reward_Plot(saveFig=False):

    df = pd.read_csv(f"{BASE_PATH}/crazyflie_projects/DeepRL/Data_Analysis/DeepRL_Training_SAC_08_09-18_18_RewardData.csv")

    fig = plt.figure(figsize=(6, 3))
    ax = fig.add_subplot(111)
    ax.plot(df["Episode"].to_numpy(),df["Value"].to_numpy(),color="tab:orange")


    ax.set_xlabel("Episode")
    ax.set_ylabel("Reward")
    ax.set_title("Reward vs Training Episode")

    ax.set_ylim(0,1)
    ax.grid()

    plt.tight_layout()
    if saveFig==True:
        plt.savefig(f'NL_DeepRL_Reward_Plot.pdf',dpi=300)

    plt.show()

if __name__ == '__main__':
    Reward_Plot(saveFig=True)