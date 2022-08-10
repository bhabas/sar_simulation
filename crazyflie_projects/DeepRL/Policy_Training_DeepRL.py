## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib as mpl
from scipy.interpolate import griddata
import plotly.graph_objects as go
import os
from datetime import datetime

## PYTORCH IMPROTS
import torch as th

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_logging.data_analysis.Data_Analysis import DataFile

from crazyflie_env.CrazyflieEnv_DeepRL import CrazyflieEnv_DeepRL
from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau
from CF_Env_2D_Simple import CF_Env_2D_Simple

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")



class Policy_Trainer_DeepRL():
    def __init__(self,env,model,model_initials):
        self.env = env
        self.model = model
        self.model_initials = model_initials

    def save_NN_Params(self,SavePath):

        FileName = f"NN_Layers_{model_initials}_DeepRL.h"
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

        # CAP the standard deviation of the actor
        LOG_STD_MAX = 2
        LOG_STD_MIN = -20
        actor = self.model.policy.actor
        obs = th.FloatTensor([obs])
        sub_action = self.model.policy.actor.forward(obs)

        ## PASS OBS THROUGH NN
        latent_pi = actor.latent_pi(obs)
        mean_actions = actor.mu(latent_pi)
        log_std = actor.log_std(latent_pi)
        log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)

        ## CONVERT MU/LOG_STD TO NORMAL DISTRIBUTION AND SAMPLE
        action_std = th.ones_like(mean_actions) * log_std.exp()
        action_mean = mean_actions

        ## CENTRAL LIMIT THEOREM SAMPLE
        normal_sample = np.sum(np.random.uniform(size=(12,2)),axis=0)-6
        samples = normal_sample*action_std.detach().numpy()+action_mean.detach().numpy()
        scaled_action = np.tanh(samples)

        ## SQUISH SAMPLES TO [-1,1] RANGE AND RESCALE
        low, high = self.env.action_space.low, self.env.action_space.high
        return low + (0.5 * (scaled_action + 1.0) * (high - low))

    def policy_dist(self,obs):

        # CAP the standard deviation of the actor
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

    def test_policy(self):
        episodes = 50
        self.env.RENDER = True
        for ep in range(episodes):
            obs = self.env.reset()
            done = False
            while not done:
                self.env.render()
                # action = custom_predict(obs)[0]
                action,_ = self.model.predict(obs)
                obs,reward,done,info = self.env.step(action)

        self.env.close()

    def train_model(self,log_name,reset_timesteps=True):

        class CheckpointSaveCallback(BaseCallback):

            def __init__(self, save_freq: int, log_dir: str, log_name: str, verbose: int = 0):
                super(CheckpointSaveCallback, self).__init__(verbose)
                self.save_freq = save_freq

                ## DIRECTORIES
                self.log_dir = os.path.join(log_dir,log_name+"_1")
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
        
        checkpoint_callback = CheckpointSaveCallback(save_freq=500,log_dir=log_dir,log_name=log_name)
        self.model.learn(
            total_timesteps=1e6,
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



if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CrazyflieEnv_DeepRL(GZ_Timeout=True)
    # env = None
    model_initials = "NL"
    log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/CF_Gazebo"


    ## LOAD MODEL
    log_name = f"SAC-08_09-18:18_1"
    policy_path = os.path.join(log_dir,log_name)
    model_path = os.path.join(log_dir,log_name,f"models/{33}500_steps.zip")
    model = SAC.load(model_path,env=env,device='cpu')
    model.load_replay_buffer(f"{log_dir}/{log_name}/models/replay_buff.pkl")
    
    


    # ## CREATE NEW MODEL 
    # log_name = f"SAC-{current_time}"
    # model = SAC(
    #     "MlpPolicy",
    #     env=env,
    #     gamma=0.999,
    #     learning_rate=0.001,
    #     policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=[12,12]),
    #     verbose=1,
    #     device='cpu',
    #     tensorboard_log=log_dir
    # ) 

    
    Policy = Policy_Trainer_DeepRL(env,model,model_initials)
    Policy.train_model(log_name,reset_timesteps=False)
    # Policy.test_policy()
    # Policy.save_NN_Params(policy_path)
    # Policy.plotPolicyRegion(iso_level=2.0)


    # dataPath = f"{BASE_PATH}/crazyflie_logging/local_logs/"
    # fileName = "Control_Playground--trial_24--NL.csv"
    # trial = DataFile(dataPath,fileName,dataType='SIM')
    # k_ep = 0
    # Policy.plotPolicyRegion(PlotTraj=(trial,k_ep,0))

    