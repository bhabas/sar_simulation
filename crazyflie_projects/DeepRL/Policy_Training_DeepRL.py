## STANDARD IMPORTS
import os
from datetime import datetime
import numpy as np
import pandas as pd
import torch as th

## PLOTTING IMPORTS
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
import plotly.graph_objects as go
from scipy.interpolate import griddata

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)


## IMPORT ENVIRONMENTS
from envs.CrazyflieEnv_DeepRL import CrazyflieEnv_DeepRL
from envs.CF_Env_2D import CF_Env_2D


## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")



class Policy_Trainer_DeepRL():
    def __init__(self,env,model,leg_config):
        self.env = env
        self.model = model
        self.leg_config = leg_config

    def save_NN_Params(self,SavePath):
        """Save NN parameters to C header file for upload to crazyflie

        Args:
            SavePath (str): Path to save header file
        """        

        FileName = f"NN_Layers_{leg_config}_DeepRL.h"
        f = open(os.path.join(SavePath,FileName),'a')
        f.truncate(0) ## Clears contents of file

        date_time = datetime.now().strftime('%m/%d-%H:%M')
        f.write(f"// Filename: {FileName} Time: {date_time}\n")
        f.write("static char NN_Params_DeepRL[] = {\n")
        
        NN_size = np.array([3]).reshape(-1,1)

        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,NN_size,
                    fmt='"%.0f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{NN_size.shape[0]},"\t"{NN_size.shape[1]},"',
                    footer='"*"\n')

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = np.zeros(3).reshape(-1,1)
        scaler_stds = np.ones(3).reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        ## SAVE PARAMETERS OF LATENT_PI LAYERS
        for module in self.model.actor.latent_pi.modules():
            if isinstance(module, th.nn.modules.linear.Linear):
                W = module.weight.detach().numpy()
                np.savetxt(f,W,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                    footer='"*"\n')


                b = module.bias.detach().numpy().reshape(-1,1)
                np.savetxt(f,b,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                    footer='"*"\n')

        ## SAVE PARAMETERS FOR MU/LOG_STD LAYER
        for module in self.model.actor.mu.modules():
            W_mu = module.weight.detach().numpy()
            b_mu = module.bias.detach().numpy().reshape(-1,1)

        for module in self.model.actor.log_std.modules():
            W_log_std = module.weight.detach().numpy()
            b_log_std = module.bias.detach().numpy().reshape(-1,1)

        ## STACK WEIGHTS AND BIASES TO MAKE ONE COHESIVE LAYER INSTEAD OF SB3 DEFAULT SPLIT
        W = np.vstack((W_mu,W_log_std))
        b = np.vstack((b_mu,b_log_std))


        np.savetxt(f,W,
            fmt='"%.5f,"',
            delimiter='\t',
            comments='',
            header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
            footer='"*"\n')

        np.savetxt(f,b,
            fmt='"%.5f,"',
            delimiter='\t',
            comments='',
            header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
            footer='"*"\n')


        f.write("};")
        f.close()

    def action_predict(self,obs): 
        """Calculate action distribution parameters through NN and then sample distributions
        for specific actions taken

        Args:
            obs (_type_): Observation vector returned by then environment

        Returns:
            _type_: Action vector
        """        

        # CAP THE STANDARD DEVIATION OF THE ACTOR
        LOG_STD_MAX = 2
        LOG_STD_MIN = -20
        actor = self.model.policy.actor
        obs = th.FloatTensor([obs])
        sub_action = self.model.policy.actor.forward(obs)

        ## PASS OBS THROUGH NN FOR DISTRIBUTION PARAMETERS (MEAN/STD DEV)
        latent_pi = actor.latent_pi(obs)
        action_means = actor.mu(latent_pi) # Values for action dist. means

        log_std = actor.log_std(latent_pi)
        log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        action_std = log_std.exp() * th.ones_like(action_means) # Values for action dist. std dev

        ## SAMPLE FROM DISTRIBUTIONS VIA CENTRAL LIMIT THEOREM
        normal_sample = np.sum(np.random.uniform(size=(12,len(action_means))),axis=0) - 6       # Sample from standard normal dist
        samples = normal_sample * action_std.detach().numpy() + action_means.detach().numpy()   # Scale sample to action dist

        ## SQUISH SAMPLES TO [-1,1] RANGE AND RESCALE
        scaled_action = np.tanh(samples)
        low, high = self.env.action_space.low, self.env.action_space.high
        return low + (0.5 * (scaled_action + 1.0) * (high - low))

    def policy_dist(self,obs):
        """Returns action distribution parameters for a given observation

        Args:
            obs (_type_): Observation vector returned by then environment

        Returns:
            action_mean: Vector for means of action distributions
            action_std: Vector for standard deviations of action distributions
        """        

        # CAP THE STANDARD DEVIATION OF THE ACTOR
        LOG_STD_MAX = 2
        LOG_STD_MIN = -20
        actor = self.model.policy.actor
        obs = th.FloatTensor([obs])

        ## PASS OBS THROUGH NN
        latent_pi = actor.latent_pi(obs)
        mean_actions = actor.mu(latent_pi)
        log_std = actor.log_std(latent_pi)
        log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)

        ## CONVERT MU/LOG_STD TO NORMAL DISTRIBUTION AND SAMPLE
        action_std = th.ones_like(mean_actions) * log_std.exp()
        action_mean = mean_actions

        action_std = action_std.detach().numpy()[0]
        action_mean = action_mean.detach().numpy()[0]


        return action_mean,action_std

    def test_policy(self,vel,phi,episodes=1):
        """Test the currently loaded policy for a given set of velocity and launch angle conditions

        Args:
            vel (float, optional): Flight velocity [m/s]. 
            phi (float, optional): Flight angle [deg]. 
            episodes (int, optional): Number of landing tests. Defaults to 1.
        """        

        for ep in range(episodes):
            obs = self.env.reset(vel=vel,phi=phi)
            done = False
            while not done:
                self.env.render()
                # action = custom_predict(obs)[0]
                action,_ = self.model.predict(obs)
                obs,reward,done,info = self.env.step(action)

    def train_model(self,log_name,reset_timesteps=True):
        """Script to train model via Deep RL method

        Args:
            log_name (str): _description_
            reset_timesteps (bool, optional): Reset starting timestep to zero. Defaults to True. 
            Set to False to resume training from previous model.
        """        

        class CheckpointSaveCallback(BaseCallback):

            def __init__(self, save_freq: int, log_dir: str, log_name: str, verbose: int = 0):
                super(CheckpointSaveCallback, self).__init__(verbose)
                self.save_freq = save_freq

                ## DIRECTORIES
                self.log_dir = os.path.join(log_dir,log_name+"_0")
                self.model_dir = os.path.join(self.log_dir,"models")
                self.replay_dir = self.log_dir

            def _init_callback(self) -> None:
                
                ## CREATE FOLDER IF NEEDED
                if self.log_dir is not None:
                    os.makedirs(self.log_dir, exist_ok=True)
                    os.makedirs(self.model_dir,exist_ok=True)

            def _on_step(self) -> bool:
                ## SAVE MODEL AND REPLAY BUFFER ON SAVE_FREQ
                if self.n_calls % self.save_freq == 0:
                    model_path = os.path.join(self.model_dir, f"{self.num_timesteps}_steps")
                    self.model.save(model_path)

                    replay_buff_path = os.path.join(self.model_dir, f"replay_buff")
                    self.model.save_replay_buffer(replay_buff_path)

                    if self.verbose > 1:
                        print(f"Saving model checkpoint to {model_path}")
                return True

            def _on_rollout_end(self) -> None:
                self.logger.record('time/K_ep',self.training_env.envs[0].env.k_ep)
                return True
        
        checkpoint_callback = CheckpointSaveCallback(save_freq=1000,log_dir=log_dir,log_name=log_name)
        self.model.learn(
            total_timesteps=2e6,
            tb_log_name=log_name,
            callback=checkpoint_callback,
            reset_num_timesteps=reset_timesteps
        )

    def plotPolicyRegion(self,PlotTraj=(None,0,0),iso_level=2.0):

        fig = go.Figure()

        ## MESHGRID OF DATA POINTS
        Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
            np.linspace(0.15, 0.30, 15),
            np.linspace(-15, 1, 15),
            np.linspace(0.0, 1.0, 15)
        )

        ## CONCATENATE DATA TO MATCH INPUT SHAPE
        X_grid = np.stack((
            Tau_grid.flatten(),
            OF_y_grid.flatten(),
            d_ceil_grid.flatten()),axis=1)


        ## CALC INDEX FOR WHERE FLIP MEAN GREATER THAN FLIP REQUIREMENT
        action_mean,_ = self.policy_dist(X_grid)
        valid_idx = np.where(action_mean[:,0] >= iso_level)


        # PLOT DATA POINTS
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=OF_y_grid.flatten()[valid_idx],
                y=Tau_grid.flatten()[valid_idx],
                z=d_ceil_grid.flatten()[valid_idx],
                

                ## MARKER
                mode='markers',
                marker=dict(
                    size=3,
                    color=action_mean[:,0].flatten()[valid_idx],
                    colorscale='Jet',   # choose a colorscale
                    colorbar=dict(title='Flip Dist Mean',),
                    cmin=1,
                    cmax=3, 
                    opacity=0.8)
            )
        )
        

        if PlotTraj[0] != None:
            dataFile,k_ep,k_run = PlotTraj
            arr = dataFile.grab_stateData(k_ep,k_run,['Tau','OF_y','d_ceil'])
            Tau,OFy,d_ceil = np.split(arr,3,axis=1)
            Tau_tr,OFy_tr,d_ceil_tr,My_tr = dataFile.grab_flip_state(k_ep,k_run,['Tau','OF_y','d_ceil','My'])

            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=OFy.flatten(),
                    y=Tau.flatten(),
                    z=d_ceil.flatten(),

                    ## MARKER
                    mode='lines',
                    marker=dict(
                        color='darkblue',
                        size=0,
                    )
                )
            )

            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=[OFy_tr],
                    y=[Tau_tr],
                    z=[d_ceil_tr],

                    ## HOVER DATA
                    hovertemplate=
                        f"<b>My: {My_tr:.3f} N*mm</b> \
                        <br>Tau: {Tau_tr:.3f} | OFy: {OFy_tr:.3f} </br> \
                        <br>D_ceil: {d_ceil_tr:.3f}</br>",

                    ## MARKER
                    mode='markers',
                    marker=dict(
                        size=3,
                        color='darkblue',
                    )
                )
            )

        

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-20,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
            ),
        )
        fig.show()

    def plot_polar(self,df,saveFig=False):

        ## COLLECT DATA
        Vx = df.iloc[:]['vx_flip_mean']
        Vz = df.iloc[:]['vz_flip_mean']

        R = np.sqrt(Vx**2 + Vz**2)
        Theta = np.degrees(np.arctan2(Vz,Vx))
        C = df.iloc[:]['LR_4Leg']

        # SOMETHING ABOUT DEFINING A GRID
        interp_factor = 12
        ri = np.linspace(1.5,3.5,(len(C)//interp_factor))
        thetai = np.linspace(25,90,(len(C)//interp_factor))
        r_ig, theta_ig = np.meshgrid(ri, thetai)
        zi = griddata((R, Theta), C, (ri[None,:], thetai[:,None]), method='linear',fill_value=0.85)
        zi = zi + 0.0001
        

        ## INIT PLOT INFO
        fig = plt.figure()
        ax = fig.add_subplot(projection='polar')
        

        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0.0,vmax=1)
        
        ax.contourf(np.radians(theta_ig),r_ig,zi,cmap=cmap,norm=norm,levels=30)
        # ax.scatter(np.radians(Theta),R,c=C,cmap=cmap,norm=norm)
        ax.set_thetamin(20)
        ax.set_thetamax(90)
        ax.set_rmin(0)
        ax.set_rmax(3.5)
        


        ## AXIS LABELS    
        ax.text(np.radians(7),2,'Flight Velocity (m/s)',
            rotation=15,ha='center',va='center')

        ax.text(np.radians(60),4.0,'Flight Angle (deg)',
            rotation=0,ha='left',va='center')

        if saveFig==True:
            plt.savefig(f'NL_Polar_DeepRL_LR.pdf',dpi=300)

        plt.show()



if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    # env = CrazyflieEnv_DeepRL(GZ_Timeout=True)
    env = CF_Env_2D()
    # env = None
    


    # # LOAD DEEP RL MODEL
    # load_model_name = f"SAC-10_12-11:10_0"

    # log_dir = f"{BASE_PATH}/crazyflie_projects/DeepRL/Training_Logs/CF_Env_2D"
    # leg_config = "NL"
    # log_name = f"SAC-{current_time}-{leg_config}"

    # policy_path = os.path.join(log_dir,log_name)
    # model_path = os.path.join(log_dir,load_model_name,f"models/{4}000_steps.zip")
    # model = SAC.load(model_path,env=env,device='cpu')
    # model.load_replay_buffer(f"{log_dir}/{load_model_name}/models/replay_buff.pkl")

    ## CREATE NEW DEEP RL MODEL 
    log_dir = f"{BASE_PATH}/crazyflie_projects/DeepRL/Training_Logs/CF_Env_2D"
    leg_config = "NL"
    log_name = f"SAC--{current_time}--{leg_config}"
    model = SAC(
        "MlpPolicy",
        env=env,
        gamma=0.999,
        learning_rate=0.002,
        policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=[12,12]),
        verbose=1,
        device='cpu',
        tensorboard_log=log_dir
    ) 

    
    Policy = Policy_Trainer_DeepRL(env,model,leg_config)
    Policy.train_model(log_name,reset_timesteps=False)


    # Policy.save_NN_Params(policy_path)
    # Policy.plotPolicyRegion(iso_level=1.5)

    # # LOAD DATA
    # df_raw = pd.read_csv(f"{BASE_PATH}/crazyflie_projects/DeepRL/Data_Logs/DeepRL_NL_LR.csv").dropna() # Collected data

    # ## MAX LANDING RATE DATAFRAME
    # idx = df_raw.groupby(['vel_d','phi_d'])['LR_4Leg'].transform(max) == df_raw['LR_4Leg']
    # df_max = df_raw[idx].reset_index()

    # # Policy.plot_polar(df_max,saveFig=False)



    # dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/"
    # fileName = "DeepRL--NL_2.75_30.00_1.csv"
    # trial = DataFile(dataPath,fileName,dataType='SIM')
    # k_ep = 0
    # Policy.plotPolicyRegion(PlotTraj=(trial,k_ep,0))

    