## STANDARD IMPORTS
import os
from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import csv
import time 

## PLOTTING IMPORTS
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib as mpl
import plotly.graph_objects as go
from scipy.interpolate import griddata

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common.vec_env import VecCheckNan
from stable_baselines3.common import utils

## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,BASE_PATH)




## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")

class CheckpointSaveCallback(BaseCallback):

    def __init__(self, save_freq: int, model_dir, verbose: int = 0):
        super(CheckpointSaveCallback, self).__init__(verbose)
        self.save_freq = save_freq
        self.model_dir = model_dir

        self.t_step_CircBuff = [0,0,0,0,0] # Circular buffer of recent timesteps

        
    def _on_step(self) -> bool:

        ## SAVE MODEL AND REPLAY BUFFER ON SAVE_FREQ
        if self.num_timesteps % self.save_freq == 0:

            ## APPEND NEWEST TIMESTEP
            self.t_step_CircBuff.append(self.num_timesteps)

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            newest_model = os.path.join(self.model_dir, f"{self.num_timesteps}_step_model")
            newest_replay_buff = os.path.join(self.model_dir, f"{self.num_timesteps}_step_replay_buff")

            self.model.save(newest_model)
            self.model.save_replay_buffer(newest_replay_buff)


            ## DELETE OLDEST MODEL AND REPLAY BUFFER
            oldest_model = os.path.join(self.model_dir,f"{self.t_step_CircBuff[0]}_step_model.zip")
            oldest_replay_buff = os.path.join(self.model_dir,f"{self.t_step_CircBuff[0]}_step_replay_buff.pkl")

            if os.path.exists(oldest_model):
                os.remove(oldest_model)

            if os.path.exists(oldest_replay_buff):
                os.remove(oldest_replay_buff)

            ## REMOVE OLDEST TIMESTEP
            self.t_step_CircBuff.pop(0)


            if self.verbose > 1:
                print(f"Saving model checkpoint to {newest_model}")
        return True

    def _on_rollout_end(self) -> None:
        self.logger.record('time/K_ep',self.training_env.envs[0].env.k_ep)
        return True


class Policy_Trainer_DeepRL():
    def __init__(self,env,log_dir,log_name):
        self.env = env
        self.log_dir = log_dir

        ## LOADED MODEL
        if log_name[-2] == "_": 
            self.log_name = log_name[:-2]
            latest_run_id = utils.get_latest_run_id(log_dir,log_name)
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_{latest_run_id}")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## NEW MODEL
        else: 
            self.log_name = log_name
            latest_run_id = utils.get_latest_run_id(log_dir,log_name)
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_{latest_run_id}")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## GENERATE LOG/MODEL DIRECTORY
        if not os.path.exists(self.TB_log_path):
            os.makedirs(self.TB_log_path,exist_ok=True)

        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir,exist_ok=True)

    def create_model(self):
        """Creates SAC agent used in DeepRL trainingg process
        """        

        self.model = SAC(
            "MlpPolicy",
            env=self.env,
            gamma=0.999,
            learning_rate=0.002,
            policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=[12,12]),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_dir
        ) 

        self.save_Config_File()

    def load_model(self,t_step):
        """Loads current model and replay buffer from the selected time step

        Args:
            t_step (int): Timestep of the selected model
        """        

        ## MODEL PATHS
        model_path = os.path.join(self.model_dir,f"{t_step}_step_model")
        replay_buff_path = os.path.join(self.model_dir,f"{t_step}_step_replay_buff")

        ## LOAD MODEL AND REPLAY BUFFER
        self.model = SAC.load(
            model_path,
            env=self.env,
            device='cpu',
            tensorboard_log=self.log_dir
        )
        self.model.load_replay_buffer(replay_buff_path)


    def save_NN_Params(self,SavePath):
        """Save NN parameters to C header file for upload to crazyflie

        Args:
            SavePath (str): Path to save header file
        """        

        FileName = f"NN_Layers_{self.env.modelInitials}_DeepRL.h"
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

    def train_model(self,total_timesteps=2e6,save_freq=1000,reset_timesteps=False):
        """Script to train model via Deep RL method

        Args:
            log_name (str): _description_
            reset_timesteps (bool, optional): Reset starting timestep to zero. Defaults to True. 
            Set to False to resume training from previous model.
        """       

        checkpoint_callback = CheckpointSaveCallback(save_freq=save_freq,model_dir=self.model_dir)
        self.model.learn(
            total_timesteps=total_timesteps,
            tb_log_name=self.log_name,
            callback=checkpoint_callback,
            reset_num_timesteps=reset_timesteps
        )

    def plotPolicyRegion(self,PlotTraj=(None,0,0),iso_level=2.0):

        fig = go.Figure()

        ## MESHGRID OF DATA POINTS
        Tau_grid, Theta_x_grid, D_perp_grid = np.meshgrid(
            np.linspace(0.15, 0.30, 15),
            np.linspace(0, 15, 15),
            np.linspace(0.0, 1.0, 15)
        )

        ## CONCATENATE DATA TO MATCH INPUT SHAPE
        X_grid = np.stack((
            Tau_grid.flatten(),
            Theta_x_grid.flatten(),
            D_perp_grid.flatten()),axis=1)


        ## CALC INDEX FOR WHERE FLIP MEAN GREATER THAN FLIP REQUIREMENT
        action_mean,_ = self.policy_dist(X_grid)
        valid_idx = np.where(action_mean[:,0] >= iso_level)


        # PLOT DATA POINTS
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=Theta_x_grid.flatten()[valid_idx],
                y=Tau_grid.flatten()[valid_idx],
                z=D_perp_grid.flatten()[valid_idx],
                

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
            arr = dataFile.grab_stateData(k_ep,k_run,['Tau','Theta_x','D_perp'])
            Tau,Theta_x,D_perp = np.split(arr,3,axis=1)
            Tau_tr,Theta_x_tr,D_perp_tr,My_tr = dataFile.grab_flip_state(k_ep,k_run,['Tau','Theta_x','D_perp','My'])

            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=Theta_x.flatten(),
                    y=Tau.flatten(),
                    z=D_perp.flatten(),

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
                    x=[Theta_x_tr],
                    y=[Tau_tr],
                    z=[D_perp_tr],

                    ## HOVER DATA
                    hovertemplate=
                        f"<b>My: {My_tr:.3f} N*mm</b> \
                        <br>Tau: {Tau_tr:.3f} | Theta_x: {Theta_x_tr:.3f} </br> \
                        <br>D_perp: {D_perp_tr:.3f}</br>",

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
                xaxis_title='Theta_x [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_perp [m]',
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

        ## COLLECT DATA
        R = df.iloc[:]['vel_d']
        Theta = df.iloc[:]['phi_d']
        C = df.iloc[:]['LR_4Leg']

        ## DEFINE INTERPOLATION GRID
        R_list = np.linspace(R.min(),R.max(),num=25,endpoint=True).reshape(1,-1)
        Theta_list = np.linspace(Theta.min(),Theta.max(),num=25,endpoint=True).reshape(1,-1)
        R_grid, Theta_grid = np.meshgrid(R_list, Theta_list)
        
        ## INTERPOLATE DATA
        LR_interp = griddata((R, Theta), C, (R_list, Theta_list.T), method='linear',fill_value=0.0)
        LR_interp += 0.001
        

        ## INIT PLOT INFO
        fig = plt.figure(figsize=(4,4))
        ax = fig.add_subplot(projection='polar')
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        
        ax.contourf(np.radians(Theta_grid),R_grid,LR_interp,cmap=cmap,norm=norm,levels=30)
        # ax.scatter(np.radians(Theta),R,c=C,cmap=cmap,norm=norm)
        # ax.scatter(np.radians(Theta_grid).flatten(),R_grid.flatten(),c=LR_interp.flatten(),cmap=cmap,norm=norm)

        ax.set_thetamin(30)
        ax.set_thetamax(90)
        ax.set_xticks(np.radians([30,45,60,75,90]))
        ax.set_rmin(0)
        ax.set_rmax(3.5)
        ax.set_rticks([0.0,1.0,2.0,3.0,3.5])
        


        ## AXIS LABELS    
        # ax.text(np.radians(7.5),2,'Flight Velocity (m/s)',
        #     rotation=18,ha='center',va='center')

        # ax.text(np.radians(60),4.5,'Flight Angle (deg)',
        #     rotation=0,ha='left',va='center')

        if saveFig==True:
            plt.savefig(f'NL_Polar_DeepRL_LR.pdf',dpi=300)

        plt.show()

    def save_Config_File(self):

        config_path = os.path.join(self.TB_log_path,"Config.yaml")

        data = dict(
            PLANE_SETTINGS = dict(
                Plane_Model = self.env.Plane_Model,
                Plane_Angle = self.env.Plane_Angle,
                Plane_Pos = dict(
                    X = self.env.Plane_Pos[0],
                    Y = self.env.Plane_Pos[1],
                    Z = self.env.Plane_Pos[2]
                ),
            ),

            QUAD_SETTINGS = dict(
                CF_Type = self.env.CF_Type,
                CF_Config = self.env.CF_Config,
            ),

            ENV_SETTINGS = dict(
                Environment = self.env.env_name,
                Vel_Limts = self.env.Vel_range,
                Phi_Limits = self.env.Phi_range,
            ),

            LEARNING_MODEL = dict(
                Policy = self.model.policy.__class__.__name__,
                Network_Layers = self.model.policy.net_arch,
                Gamma = self.model.gamma,
                Learning_Rate = self.model.learning_rate,
                Activation_Function = "",
            )


        )

        with open(config_path, 'w') as outfile:
            yaml.dump(data,outfile,default_flow_style=False,sort_keys=False)

    def test_landing_performance(self,fileName=None,Vel_inc=0.25,Phi_inc=5,n_episodes=20):
        """Test trained model over varied velocity and flight angle combinations.
        Args:
            fileName (str, optional): fileName to save logged CSV as. Defaults to None.
            Vel_inc (float, optional): Flight velocity increment [m/s]. Defaults to 0.25.
            Phi_inc (int, optional): Flight angle increment [deg]. Defaults to 5.
            n_episodes (int, optional): Number of episodes to test each velocity over. Defaults to 5.
        """        

        if fileName == None:
            fileName = "ThrowAwayValuess.csv"
        filePath = os.path.join(self.TB_log_path,fileName)

        Vel_arr = np.arange(self.env.Vel_range[0], self.env.Vel_range[1] + Vel_inc, Vel_inc)
        Phi_arr = np.arange(self.env.Phi_range[0], self.env.Phi_range[1] + Phi_inc, Phi_inc)

        

        def EMA(cur_val,prev_val,alpha = 0.15):
            """Exponential Moving Average Filter
            Args:
                cur_val (float): Current Value
                prev_val (float): Previous Value
                alpha (float, optional): Smoothing factor. Similar to moving average window (k), 
                by alpha = 2/(k+1). Defaults to 0.15.
            Returns:
                float: Current average value
            """            
            
            return alpha*cur_val + (1-alpha)*(prev_val)


        ## TIME ESTIMATION FILTER INITIALIZATION
        num_trials = len(Vel_arr)*len(Phi_arr)*n_episodes
        idx = 0
        t_delta = 0
        t_delta_prev = 0

        ## WRITE FILE HEADER IF NO FILE EXISTS
        if not os.path.exists(filePath):
            with open(filePath,'w') as file:
                writer = csv.writer(file,delimiter=',')
                writer.writerow([
                    "Vel_d", "Phi_d", "Trial_num",
                    "--",
                    "Pad_Connections","Body_Contact",

                    "Vel_flip","Phi_flip",

                    "Tau_tr",
                    "Theta_x_tr",
                    "D_perp_tr",

                    "Eul_y_Impact",
                    
                    "Policy_tr",
                    "Policy_action",

                    "Vx_tr",
                    "Vz_tr",
                    "reward","reward_vals",
                    "--",
                    "4_Leg_NBC","4_Leg_BC",
                    "2_Leg_NBC","2_Leg_BC",
                    "0_Leg_NBC","0_Leg_BC",


                ])


        for Vel in Vel_arr:
            for Phi in Phi_arr:
                for K_ep in range(n_episodes):

                    start_time = time.time()

                    ## TEST POLICY FOR GIVEN FLIGHT CONDITIONS
                    # original, which errs on vel. 
                    obs = self.env.reset(vel=Vel,phi=Phi)
                    # obs = self.env.reset()
                    done = False
                    while not done:
                        action,_ = self.model.predict(obs)
                        #action = np.zeros_like(action)
                        obs,reward,done,info = self.env.step(action)


                    PC = self.env.pad_connections
                    BC = self.env.BodyContact_flag

                    if   PC >= 3 and BC == False:       # 4_Leg_NBC
                        LS = [1,0,0,0,0,0]
                    elif PC >= 3 and BC == True:        # 4_Leg_BC
                        LS = [0,1,0,0,0,0]
                    elif PC == 2 and BC == False:       # 2_Leg_NBC
                        LS = [0,0,1,0,0,0]
                    elif PC == 2 and BC == True:        # 2_Leg_BC
                        LS = [0,0,0,1,0,0]
                    elif PC <= 1 and BC == False:       # 0_Leg_NBC
                        LS = [0,0,0,0,1,0]
                    elif PC <= 1 and BC == True:        # 0_Leg_BC
                        LS = [0,0,0,0,0,1]                 

                            
                    ## APPEND RECORDED VALUES TO FILE
                    with open(filePath,'a') as file:
                        writer = csv.writer(file,delimiter=',',quoting=csv.QUOTE_NONE)
                        writer.writerow([
                            np.round(Vel,2),np.round(Phi,2),K_ep,
                            "--",
                            self.env.pad_connections,self.env.BodyContact_flag,

                            np.round(self.env.vel_tr_mag,2),np.round(self.env.phi_tr,2),

                            np.round(self.env.obs_tr[0],3),
                            np.round(self.env.obs_tr[1],3),
                            np.round(self.env.obs_tr[2],3),

                            np.round(self.env.eul_impact[1],3),

                            np.round(self.env.action_tr[0],3),
                            np.round(self.env.action_tr[1],3),

                            np.round(self.env.vel_tr[0],3),
                            np.round(self.env.vel_tr[2],3),

                            np.round(reward,3),
                            np.round(self.env.reward_vals,3),
                            "--",
                            LS[0],LS[1],
                            LS[2],LS[3],
                            LS[4],LS[5],
                        ])

                        ## CALCULATE AVERAGE TIME PER EPISODE
                        t_delta = time.time() - start_time
                        t_delta_avg = EMA(t_delta,t_delta_prev,alpha=0.01)
                        t_delta_prev = t_delta_avg
                        idx += 1

                        TTC = round(t_delta_avg*(num_trials-idx)) # Time to completion
                        print(f"Flight Conditions: ({Vel} m/s,{Phi} deg) \t Index: {idx}/{num_trials} \t Percentage: {100*idx/num_trials:.2f}% \t Time to Completion: {str(timedelta(seconds=TTC))}")

    def Plot_Landing_Performance(self,fileName=None,saveFig=False):

        if fileName == None:
            fileName = "PolicyPerformance_Data_20Trials.csv"
        filePath = os.path.join(self.TB_log_path,fileName)

        af = pd.read_csv(filePath,sep=',',comment="#")

        af2 = af.groupby(['Vel_d','Phi_d']).mean().round(3).reset_index()



        ## COLLECT DATA
        R = af2.iloc[:]['Vel_d']
        Theta = af2.iloc[:]['Phi_d']
        C = af2.iloc[:]['4_Leg_BC']
        # C1 = af2.iloc[:]['2_Leg_NBC']
        # C2 = af2.iloc[:]['2_Leg_BC']
        # C = np.add(C1,C2)


        ## DEFINE INTERPOLATION GRID
        R_list = np.linspace(R.min(),R.max(),num=50,endpoint=True).reshape(1,-1)
        Theta_list = np.linspace(Theta.min(),Theta.max(),num=50,endpoint=True).reshape(1,-1)
        R_grid, Theta_grid = np.meshgrid(R_list, Theta_list)
        
        ## INTERPOLATE DATA
        LR_interp = griddata((R, Theta), C, (R_list, Theta_list.T), method='linear')
        LR_interp += 0.001
        

        ## INIT PLOT INFO
        fig = plt.figure(figsize=(6,6))
        ax = fig.add_subplot(projection='polar')
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        
        ax.contourf(np.radians(Theta_grid),R_grid,LR_interp,cmap=cmap,norm=norm,levels=30)
        # ax.scatter(np.radians(Theta),R,c=C,cmap=cmap,norm=norm)
        # ax.scatter(np.radians(Theta_grid).flatten(),R_grid.flatten(),c=LR_interp.flatten(),cmap=cmap,norm=norm)

        ax.set_xticks(np.radians(np.arange(-90,90+15,15)))
        ax.set_thetamin(Theta.min())
        ax.set_thetamax(Theta.max())

        ax.set_rticks([0.0,1.0,2.0,3.0,4.0,4.5])
        ax.set_rmin(0)
        ax.set_rmax(R.max())
        


        # ## AXIS LABELS    
        # ax.text(np.radians(7.5),2,'Flight Velocity (m/s)',
        #     rotation=18,ha='center',va='center')

        # ax.text(np.radians(60),4.5,'Flight Angle (deg)',
        #     rotation=0,ha='left',va='center')

        if saveFig==True:
            plt.savefig(f'{self.TB_log_path}/Landing_Rate_Fig_20Trials_4BC.pdf',dpi=300)

        plt.show()


    def Configuration_Comparison(self,fileName=None,saveFig=True):
        avg = []
        avgBC4 = []
        avgLeg2 = []
        avgNoLeg = []

        secar =[]
        secavgar =[] #Array for section average
        secarBC4 =[]
        secavgarBC4 =[] #Array for section average
        secarLeg2 =[]
        secavgarLeg2 =[] #Array for section average
        secarNoLeg =[]
        secavgarNoLeg =[] #Array for section average


        angle = [60, 60, 60, 60, 60, 60, 60, 60, 60, 30, 30, 30, 30, 30, 30, 30, 30, 30, 5, 5, 5, 5, 5, 5, 5, 5, 5]
        length = [100, 100, 100, 75, 75, 75, 50, 50, 50, 100, 100, 100, 75, 75, 75, 50, 50, 50, 100, 100, 100, 75, 75, 50, 50, 75, 50]
        stiffness = [64, 32, 8, 64, 32, 8, 64, 32, 8, 64, 32, 8, 64, 32, 8, 64, 32, 8, 64, 32, 8, 32, 64, 64, 32, 8, 8]
          
        # self.log_name = log_name[:-2]
        # latest_run_id = utils.get_latest_run_id(log_dir,log_name)
        # self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_{latest_run_id}")
        # self.model_dir = os.path.join(self.TB_log_path,"models")
        for filename in sorted(os.listdir(self.log_dir)):
            f = os.path.join(self.log_dir, filename, self.log_name)
            if filename == "PolicyPerformance_Data_20Trials.csv_0":
                os.rmdir(os.path.join(self.log_dir, filename, "models"))
                os.rmdir(os.path.join(self.log_dir, filename)) 
            else:
                af = pd.read_csv(f,sep=',',comment="#")

                af2 = af.groupby(['Vel_d','Phi_d']).mean().round(3).reset_index()

                ## COLLECT DATA
                # R = af2.iloc[:]['Vel_d']
                # Theta = af2.iloc[:]['Phi_d']
                #C = af2.iloc[:]['4_Leg_NBC']
                C = af2['4_Leg_NBC'].to_numpy()
                A = np.mean(C)
                avg.append(A)

                for x in range(0, 16):
                    secar = []
                    for y in range(0, 4):
                        secbit = []
                        i = round(y*13+(x%4)*3+np.floor(x/4)*39)
                        secbit = C[i:(i+4)]
                        secar = np.concatenate([secar, secbit])
                    secavg = np.mean(secar)
                    secavgar.append(secavg)



                BC4 = af2.iloc[:]['4_Leg_BC']
                ABC4 = np.mean(BC4)
                avgBC4.append(ABC4)

                for xBC4 in range(0, 16):
                    secarBC4 = []
                    for yBC4 in range(0, 4):
                        secbitBC4 = []
                        iBC4 = round(yBC4*13+(xBC4%4)*3+np.floor(xBC4/4)*39)
                        secbitBC4 = BC4[iBC4:(iBC4+4)]
                        secarBC4 = np.concatenate([secarBC4, secbitBC4])
                    secavgBC4 = np.mean(secarBC4)
                    secavgarBC4.append(secavgBC4)

                C1 = af2.iloc[:]['2_Leg_NBC']
                C2 = af2.iloc[:]['2_Leg_BC']
                Leg2 = np.add(C1,C2)
                ALeg2 = np.mean(Leg2)
                avgLeg2.append(ALeg2)

                for xLeg2 in range(0, 16):
                    secarLeg2 = []
                    for yLeg2 in range(0, 4):
                        secbitLeg2 = []
                        iLeg2 = round(yLeg2*13+(xLeg2%4)*3+np.floor(xLeg2/4)*39)
                        secbitLeg2 = Leg2[iLeg2:(iLeg2+4)]
                        secarLeg2 = np.concatenate([secarLeg2, secbitLeg2])
                    secavgLeg2 = np.mean(secarLeg2)
                    secavgarLeg2.append(secavgLeg2)

                N1 = af2.iloc[:]['0_Leg_NBC']
                N2 = af2.iloc[:]['0_Leg_BC']
                NoLeg = np.add(N1,N2)
                ANoLeg = np.mean(NoLeg)
                avgNoLeg.append(ANoLeg)

                for xNoLeg in range(0, 16):
                    secarNoLeg = []
                    for yNoLeg in range(0, 4):
                        secbitNoLeg = []
                        iNoLeg = round(yNoLeg*13+(xNoLeg%4)*3+np.floor(xNoLeg/4)*39)
                        secbitNoLeg = NoLeg[iNoLeg:(iNoLeg+4)]
                        secarNoLeg = np.concatenate([secarNoLeg, secbitNoLeg])
                    secavgNoLeg = np.mean(secarNoLeg)
                    secavgarNoLeg.append(secavgNoLeg)
        
        fig, ax = plt.subplots(figsize=(20,15))
        ax.set_title("Success Trends By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        major_xticks=np.arange(0,1.01,0.25)
        plt.xticks(major_xticks, [90,75,60,45,30])
        ax.xaxis.set_label_coords(0.5, 1.055)
        ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        major_yticks=np.arange(0,1.01,0.25)
        plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        ax.yaxis.set_label_coords(-0.035, 0.5)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)

        for saa in range(0,16):
            samplesaa = []

            for x in range(0,27):
                samplesaa.append(secavgar[(x*16+15-saa)])
            size = [round(num*300) for num in samplesaa]
            
            ax = fig.add_subplot(4,4,(saa+1), projection='3d')
            ax.view_init(20, -73)

            # cmap = mpl.cm.jet
            # norm = mpl.colors.Normalize(vmin=0,vmax=1)
            # ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=samplesaa, cmap = cmap, norm = norm)

            # ax.set_xlim(0,60)
            # ax.set_zlim(50,100)
            # ax.set_ylim(0,70)
            # ax.set_xlabel("Angle (deg)")
            # ax.set_zlabel("Length (mm)")
            # ax.set_ylabel("Stiffness (Nm/rad)")


            xpoints = [5,30,60]
            ypoints = [8,32,64]
            zpoints = [50,75,100]
            ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
            xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
            for i in xpoints:
                for j in ypoints:  
                    ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for i in xpoints:
                for k in zpoints:
                    if k == 50:
                        ax.plot([i,i],[ymin,ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for j in ypoints:
                for k in zpoints:
                    if j == 64 or k == 50:
                        ax.plot([xpoints[0],xmax],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            
            cmap = mpl.cm.jet
            norm = mpl.colors.Normalize(vmin=0,vmax=1)
            ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=samplesaa, cmap = cmap, norm = norm)
            ax.set_xlim(0,60)
            plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
            ax.set_zlim(50,100)
            ax.set_zticks([52,77,102])
            ax.set_zticklabels(['   50mm','   75mm','   100mm'])
            ax.set_ylim(0,70)
            plt.yticks([26,54,84], ['   80mJ/r','     320mJ/r','     640mJ/r'])
            ax.set_xlabel("Angle", fontweight = 'bold')
            ax.set_zlabel("Length       ", fontweight = 'bold')
            ax.set_ylabel("             Stiffness", fontweight='bold')
            ax.yaxis.labelpad=20
            ax.zaxis.labelpad=15
            ax.xaxis.labelpad=9

            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False
            ax.xaxis.pane.set_edgecolor('white')
            ax.yaxis.pane.set_edgecolor('white')
            ax.zaxis.pane.set_edgecolor('white')
            ax.w_xaxis.line.set_color("white")
            ax.w_yaxis.line.set_color("white")
            ax.w_zaxis.line.set_color("white")
            ax.xaxis._axinfo['tick']['color']='w'
            ax.yaxis._axinfo['tick']['color']='w'
            ax.zaxis._axinfo['tick']['color']='w'
            ax.grid(False)

        if saveFig==True:
            plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest.pdf',dpi=300)

        
        ##4 Leg Body Contact by Section
        fig, ax = plt.subplots(figsize=(20,15))
        ax.set_title("Four Leg Body Contact Failure By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        major_xticks=np.arange(0,1.01,0.25)
        plt.xticks(major_xticks, [90,75,60,45,30])
        ax.xaxis.set_label_coords(0.5, 1.055)
        ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        major_yticks=np.arange(0,1.01,0.25)
        plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        ax.yaxis.set_label_coords(-0.035, 0.5)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)

        for saa in range(0,16):
            samplesaa = []

            for x in range(0,27):
                samplesaa.append(secavgarBC4[(x*16+15-saa)])
            size = [round(num*300) for num in samplesaa]
            
            ax = fig.add_subplot(4,4,(saa+1), projection='3d')
            ax.view_init(20, -73)


            xpoints = [5,30,60]
            ypoints = [8,32,64]
            zpoints = [50,75,100]
            ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
            xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
            for i in xpoints:
                for j in ypoints:  
                    ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for i in xpoints:
                for k in zpoints:
                    if k == 50:
                        ax.plot([i,i],[ymin,ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for j in ypoints:
                for k in zpoints:
                    if j == 64 or k == 50:
                        ax.plot([xpoints[0],xmax],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            
            cmap = mpl.cm.BuPu
            norm = mpl.colors.Normalize(vmin=0,vmax=1)
            ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=[item*0.5+0.5 for item in samplesaa], cmap = cmap, norm = norm)
            ax.set_xlim(0,60)
            plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
            ax.set_zlim(50,100)
            ax.set_zticks([52,77,102])
            ax.set_zticklabels(['   50mm','   75mm','   100mm'])
            ax.set_ylim(0,70)
            plt.yticks([26,54,84], ['   80mJ/r','     320mJ/r','     640mJ/r'])
            ax.set_xlabel("Angle", fontweight = 'bold')
            ax.set_zlabel("Length       ", fontweight = 'bold')
            ax.set_ylabel("             Stiffness", fontweight='bold')
            ax.yaxis.labelpad=20
            ax.zaxis.labelpad=15
            ax.xaxis.labelpad=9

            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False
            ax.xaxis.pane.set_edgecolor('white')
            ax.yaxis.pane.set_edgecolor('white')
            ax.zaxis.pane.set_edgecolor('white')
            ax.w_xaxis.line.set_color("white")
            ax.w_yaxis.line.set_color("white")
            ax.w_zaxis.line.set_color("white")
            ax.xaxis._axinfo['tick']['color']='w'
            ax.yaxis._axinfo['tick']['color']='w'
            ax.zaxis._axinfo['tick']['color']='w'
            ax.grid(False)

        if saveFig==True:
            plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest2.pdf',dpi=300)

        
        ##2 Leg Body Failure by Section
        fig, ax = plt.subplots(figsize=(20,15))
        ax.set_title("Two Leg Failure By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        major_xticks=np.arange(0,1.01,0.25)
        plt.xticks(major_xticks, [90,75,60,45,30])
        ax.xaxis.set_label_coords(0.5, 1.055)
        ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        major_yticks=np.arange(0,1.01,0.25)
        plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        ax.yaxis.set_label_coords(-0.035, 0.5)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)

        for saa in range(0,16):
            samplesaa = []

            for x in range(0,27):
                samplesaa.append(secavgarLeg2[(x*16+15-saa)])
            size = [round(num*300) for num in samplesaa]
            
            ax = fig.add_subplot(4,4,(saa+1), projection='3d')
            ax.view_init(20, -73)


            xpoints = [5,30,60]
            ypoints = [8,32,64]
            zpoints = [50,75,100]
            ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
            xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
            for i in xpoints:
                for j in ypoints:  
                    ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for i in xpoints:
                for k in zpoints:
                    if k == 50:
                        ax.plot([i,i],[ymin,ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for j in ypoints:
                for k in zpoints:
                    if j == 64 or k == 50:
                        ax.plot([xpoints[0],xmax],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            
            cmap = mpl.cm.YlOrBr
            norm = mpl.colors.Normalize(vmin=0,vmax=1)
            ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=[item*0.5+0.5 for item in samplesaa], cmap = cmap, norm = norm)
            ax.set_xlim(0,60)
            plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
            ax.set_zlim(50,100)
            ax.set_zticks([52,77,102])
            ax.set_zticklabels(['   50mm','   75mm','   100mm'])
            ax.set_ylim(0,70)
            plt.yticks([26,54,84], ['   80mJ/r','     320mJ/r','     640mJ/r'])
            ax.set_xlabel("Angle", fontweight = 'bold')
            ax.set_zlabel("Length       ", fontweight = 'bold')
            ax.set_ylabel("             Stiffness", fontweight='bold')
            ax.yaxis.labelpad=20
            ax.zaxis.labelpad=15
            ax.xaxis.labelpad=9

            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False
            ax.xaxis.pane.set_edgecolor('white')
            ax.yaxis.pane.set_edgecolor('white')
            ax.zaxis.pane.set_edgecolor('white')
            ax.w_xaxis.line.set_color("white")
            ax.w_yaxis.line.set_color("white")
            ax.w_zaxis.line.set_color("white")
            ax.xaxis._axinfo['tick']['color']='w'
            ax.yaxis._axinfo['tick']['color']='w'
            ax.zaxis._axinfo['tick']['color']='w'
            ax.grid(False)

        if saveFig==True:
            plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest3.pdf',dpi=300)

        ##No Leg Body Failure by Section
        fig, ax = plt.subplots(figsize=(20,15))
        ax.set_title("Zero Leg Failure By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        major_xticks=np.arange(0,1.01,0.25)
        plt.xticks(major_xticks, [90,75,60,45,30])
        ax.xaxis.set_label_coords(0.5, 1.055)
        ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        major_yticks=np.arange(0,1.01,0.25)
        plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        ax.yaxis.set_label_coords(-0.035, 0.5)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)

        for saa in range(0,16):
            samplesaa = []

            for x in range(0,27):
                samplesaa.append(secavgarNoLeg[(x*16+15-saa)])
            size = [round(num*300) for num in samplesaa]
            
            ax = fig.add_subplot(4,4,(saa+1), projection='3d')
            ax.view_init(20, -73)


            xpoints = [5,30,60]
            ypoints = [8,32,64]
            zpoints = [50,75,100]
            ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
            xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
            for i in xpoints:
                for j in ypoints:  
                    ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for i in xpoints:
                for k in zpoints:
                    if k == 50:
                        ax.plot([i,i],[ymin,ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            for j in ypoints:
                for k in zpoints:
                    if j == 64 or k == 50:
                        ax.plot([xpoints[0],xmax],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
                    else:
                        ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k], color = (0.5, 0.5, 0.5), linewidth = 0.2)
            
            cmap = mpl.cm.gist_yarg
            norm = mpl.colors.Normalize(vmin=0,vmax=1)
            ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=[item*0.8+0.2 for item in samplesaa], cmap = cmap, norm = norm)
            ax.set_xlim(0,60)
            plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
            ax.set_zlim(50,100)
            ax.set_zticks([52,77,102])
            ax.set_zticklabels(['   50mm','   75mm','   100mm'])
            ax.set_ylim(0,70)
            plt.yticks([26,54,84], ['   80mJ/r','     320mJ/r','     640mJ/r'])
            ax.set_xlabel("Angle", fontweight = 'bold')
            ax.set_zlabel("Length       ", fontweight = 'bold')
            ax.set_ylabel("             Stiffness", fontweight='bold')
            ax.yaxis.labelpad=20
            ax.zaxis.labelpad=15
            ax.xaxis.labelpad=9

            ax.xaxis.pane.fill = False
            ax.yaxis.pane.fill = False
            ax.zaxis.pane.fill = False
            ax.xaxis.pane.set_edgecolor('white')
            ax.yaxis.pane.set_edgecolor('white')
            ax.zaxis.pane.set_edgecolor('white')
            ax.w_xaxis.line.set_color("white")
            ax.w_yaxis.line.set_color("white")
            ax.w_zaxis.line.set_color("white")
            ax.xaxis._axinfo['tick']['color']='w'
            ax.yaxis._axinfo['tick']['color']='w'
            ax.zaxis._axinfo['tick']['color']='w'
            ax.grid(False)

        if saveFig==True:
            plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest4.pdf',dpi=300)


        # fig, ax = plt.subplots(figsize=(20,15))
        # ax.set_title("Four Leg Body Contact Failure By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        # ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        # ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        # major_xticks=np.arange(0,1.01,0.25)
        # plt.xticks(major_xticks, [30,45,60,75,90])
        # ax.xaxis.set_label_coords(0.5, 1.055)
        # ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        # major_yticks=np.arange(0,1.01,0.25)
        # plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        # ax.yaxis.set_label_coords(-0.035, 0.5)
        # ax.spines['bottom'].set_visible(False)
        # ax.spines['right'].set_visible(False)

        # for saa in range(0,16):
        #     samplesaa = []

        #     for x in range(0,27):
        #         samplesaa.append(secavgarBC4[(x*16+15-saa)])
        #     size = [round(num*300) for num in samplesaa]
            
        #     ax = fig.add_subplot(4,4,(saa+1), projection='3d')
        #     ax.view_init(26, -73)

        #     cmap = mpl.cm.copper
        #     norm = mpl.colors.Normalize(vmin=0,vmax=1)
        #     ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=samplesaa, cmap = cmap, norm = norm)

        #     ax.set_xlim(0,60)
        #     ax.set_zlim(50,100)
        #     ax.set_ylim(0,70)
        #     ax.set_xlabel("Angle (deg)")
        #     ax.set_zlabel("Length (mm)")
        #     ax.set_ylabel("Stiffness (Nm/rad)")    
        
        # if saveFig==True:
        #     plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest2.pdf',dpi=300)


        # fig, ax = plt.subplots(figsize=(20,15))
        # ax.set_title("Two Leg Failure By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        # ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        # ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        # major_xticks=np.arange(0,1.01,0.25)
        # plt.xticks(major_xticks, [30,45,60,75,90])
        # ax.xaxis.set_label_coords(0.5, 1.055)
        # ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        # major_yticks=np.arange(0,1.01,0.25)
        # plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        # ax.yaxis.set_label_coords(-0.035, 0.5)
        # ax.spines['bottom'].set_visible(False)
        # ax.spines['right'].set_visible(False)

        # for saa in range(0,16):
        #     samplesaa = []

        #     for x in range(0,27):
        #         samplesaa.append(secavgarLeg2[(x*16+15-saa)])
        #     size = [round(num*300) for num in samplesaa]
            
        #     ax = fig.add_subplot(4,4,(saa+1), projection='3d')
        #     ax.view_init(26, -73)

        #     cmap = mpl.cm.Wistia
        #     norm = mpl.colors.Normalize(vmin=0,vmax=1)
        #     ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=samplesaa, cmap = cmap, norm = norm)

        #     ax.set_xlim(0,60)
        #     ax.set_zlim(50,100)
        #     ax.set_ylim(0,70)
        #     ax.set_xlabel("Angle (deg)")
        #     ax.set_zlabel("Length (mm)")
        #     ax.set_ylabel("Stiffness (Nm/rad)") 

        # if saveFig==True:
        #     plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest3.pdf',dpi=300)


        # fig, ax = plt.subplots(figsize=(20,15))
        # ax.set_title("Zero Leg Failure By Approach Velocity-Angle Sub-Sections", y = -0.1, fontsize = 20)
        # ax.set_xlabel("Approach Angle (deg)", fontsize = 12)
        
        # ax.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
        # major_xticks=np.arange(0,1.01,0.25)
        # plt.xticks(major_xticks, [30,45,60,75,90])
        # ax.xaxis.set_label_coords(0.5, 1.055)
        # ax.set_ylabel("Approach Velocity (m/s)", fontsize = 12)
        # major_yticks=np.arange(0,1.01,0.25)
        # plt.yticks(major_yticks, [0.5,1.25,2.0,2.75,3.5])
        # ax.yaxis.set_label_coords(-0.035, 0.5)
        # ax.spines['bottom'].set_visible(False)
        # ax.spines['right'].set_visible(False)

        # for saa in range(0,16):
        #     samplesaa = []

        #     for x in range(0,27):
        #         samplesaa.append(secavgarNoLeg[(x*16+15-saa)])
        #     size = [round(num*300) for num in samplesaa]
            
        #     ax = fig.add_subplot(4,4,(saa+1), projection='3d')
        #     ax.view_init(26, -73)

        #     cmap = mpl.cm.gist_yarg
        #     norm = mpl.colors.Normalize(vmin=0,vmax=1)
        #     ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=samplesaa, cmap = cmap, norm = norm)

        #     ax.set_xlim(0,60)
        #     ax.set_zlim(50,100)
        #     ax.set_ylim(0,70)
        #     ax.set_xlabel("Angle (deg)")
        #     ax.set_zlabel("Length (mm)")
        #     ax.set_ylabel("Stiffness (Nm/rad)") 

        # if saveFig==True:
        #     plt.savefig(f'{os.path.join(self.log_dir, filename)}/BigPlotTest4.pdf',dpi=300)
                
        ##Dot Size Calculations for Average Plots
        size = [round(num*700) for num in avg]
        sizeBC4 = [round(num*700) for num in avgBC4]
        sizeLeg2 = [round(num*700) for num in avgLeg2]
        sizeNoLeg = [round(num*700) for num in avgNoLeg]


        ##Average Success Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(25, -77)
        
        xpoints = [5,30,60]
        ypoints = [8,32,64]
        zpoints = [50,75,100]
        ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
        xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
        for i in xpoints:
            for j in ypoints:  
                ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = 'k', linewidth = 0.5)
        for i in xpoints:
            for k in zpoints:
                if k == 50:
                    ax.plot([i,i],[ymin,ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
        for j in ypoints:
            for k in zpoints:
                if j == 64 or k == 50:
                    ax.plot([xpoints[0],xmax],[j,j],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k],color = 'k', linewidth = 0.5)
        
        cmap = mpl.cm.jet
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        ax.scatter(xs = angle, ys = stiffness, zs = length, s = size, c=avg, cmap = cmap, norm = norm)
        ax.set_xlim(0,60)
        plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
        ax.set_zlim(50,100)
        ax.set_zticks([52,77,102])
        ax.set_zticklabels(['   50mm','   75mm','   100mm'])
        ax.set_ylim(0,70)
        plt.yticks([14,38,68], ['   80mJ/r','     320mJ/r','     640mJ/r'])
        ax.set_title("Average Success", fontweight = 'bold')
        ax.set_xlabel("Angle", fontweight = 'bold')
        ax.set_zlabel("Length       ", fontweight = 'bold')
        ax.set_ylabel("             Stiffness", fontweight='bold')
        ax.yaxis.labelpad=20
        ax.zaxis.labelpad=15
        ax.xaxis.labelpad=9

        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('white')
        ax.yaxis.pane.set_edgecolor('white')
        ax.zaxis.pane.set_edgecolor('white')
        ax.w_xaxis.line.set_color("white")
        ax.w_yaxis.line.set_color("white")
        ax.w_zaxis.line.set_color("white")
        ax.xaxis._axinfo['tick']['color']='w'
        ax.yaxis._axinfo['tick']['color']='w'
        ax.zaxis._axinfo['tick']['color']='w'
        ax.grid(False)


        ##4 Leg Body Contact Average
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(25, -77)
        
        xpoints = [5,30,60]
        ypoints = [8,32,64]
        zpoints = [50,75,100]
        ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
        xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
        for i in xpoints:
            for j in ypoints:  
                ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = 'k', linewidth = 0.5)
        for i in xpoints:
            for k in zpoints:
                if k == 50:
                    ax.plot([i,i],[ymin,ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
        for j in ypoints:
            for k in zpoints:
                if j == 64 or k == 50:
                    ax.plot([xpoints[0],xmax],[j,j],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k],color = 'k', linewidth = 0.5)
        
        cmap = mpl.cm.BuPu
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        ax.scatter(xs = angle, ys = stiffness, zs = length, s = sizeBC4, c= [item*0.5+0.5 for item in avgBC4], cmap = cmap, norm = norm)
        ax.set_xlim(0,60)
        plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
        ax.set_zlim(50,100)
        ax.set_zticks([52,77,102])
        ax.set_zticklabels(['   50mm','   75mm','   100mm'])
        ax.set_ylim(0,70)
        plt.yticks([14,38,68], ['   80mJ/r','     320mJ/r','     640mJ/r'])
        ax.set_title("Average 4-Leg Body Contact Failure", fontweight = 'bold')
        ax.set_xlabel("Angle", fontweight = 'bold')
        ax.set_zlabel("Length       ", fontweight = 'bold')
        ax.set_ylabel("             Stiffness", fontweight='bold')
        ax.yaxis.labelpad=20
        ax.zaxis.labelpad=15
        ax.xaxis.labelpad=9

        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('white')
        ax.yaxis.pane.set_edgecolor('white')
        ax.zaxis.pane.set_edgecolor('white')
        ax.w_xaxis.line.set_color("white")
        ax.w_yaxis.line.set_color("white")
        ax.w_zaxis.line.set_color("white")
        ax.xaxis._axinfo['tick']['color']='w'
        ax.yaxis._axinfo['tick']['color']='w'
        ax.zaxis._axinfo['tick']['color']='w'
        ax.grid(False)

        ## 2 Leg Failure Average Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(25, -77)
        
        xpoints = [5,30,60]
        ypoints = [8,32,64]
        zpoints = [50,75,100]
        ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
        xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
        for i in xpoints:
            for j in ypoints:  
                ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = 'k', linewidth = 0.5)
        for i in xpoints:
            for k in zpoints:
                if k == 50:
                    ax.plot([i,i],[ymin,ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
        for j in ypoints:
            for k in zpoints:
                if j == 64 or k == 50:
                    ax.plot([xpoints[0],xmax],[j,j],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k],color = 'k', linewidth = 0.5)
        
        cmap = mpl.cm.YlOrBr
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        ax.scatter(xs = angle, ys = stiffness, zs = length, s = sizeLeg2, c= [item*0.5+0.5 for item in avgLeg2], cmap = cmap, norm = norm)
        ax.set_xlim(0,60)
        plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
        ax.set_zlim(50,100)
        ax.set_zticks([52,77,102])
        ax.set_zticklabels(['   50mm','   75mm','   100mm'])
        ax.set_ylim(0,70)
        plt.yticks([14,38,68], ['   80mJ/r','     320mJ/r','     640mJ/r'])
        ax.set_title("Average 2-Leg Failure", fontweight = 'bold')
        ax.set_xlabel("Angle", fontweight = 'bold')
        ax.set_zlabel("Length       ", fontweight = 'bold')
        ax.set_ylabel("             Stiffness", fontweight='bold')
        ax.yaxis.labelpad=20
        ax.zaxis.labelpad=15
        ax.xaxis.labelpad=9

        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('white')
        ax.yaxis.pane.set_edgecolor('white')
        ax.zaxis.pane.set_edgecolor('white')
        ax.w_xaxis.line.set_color("white")
        ax.w_yaxis.line.set_color("white")
        ax.w_zaxis.line.set_color("white")
        ax.xaxis._axinfo['tick']['color']='w'
        ax.yaxis._axinfo['tick']['color']='w'
        ax.zaxis._axinfo['tick']['color']='w'
        ax.grid(False)


        ## No Leg Failure Average Plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(25, -77)
        
        xpoints = [5,30,60]
        ypoints = [8,32,64]
        zpoints = [50,75,100]
        ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
        xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
        for i in xpoints:
            for j in ypoints:  
                ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = 'k', linewidth = 0.5)
        for i in xpoints:
            for k in zpoints:
                if k == 50:
                    ax.plot([i,i],[ymin,ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
        for j in ypoints:
            for k in zpoints:
                if j == 64 or k == 50:
                    ax.plot([xpoints[0],xmax],[j,j],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k],color = 'k', linewidth = 0.5)
        
        cmap = mpl.cm.gist_yarg
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        ax.scatter(xs = angle, ys = stiffness, zs = length, s = sizeNoLeg, c= [item*0.8+0.2 for item in avgNoLeg], cmap = cmap, norm = norm)
        ax.set_xlim(0,60)
        plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
        ax.set_zlim(50,100)
        ax.set_zticks([52,77,102])
        ax.set_zticklabels(['   50mm','   75mm','   100mm'])
        ax.set_ylim(0,70)
        plt.yticks([14,38,68], ['   80mJ/r','     320mJ/r','     640mJ/r'])
        ax.set_title("Average No-Leg Failure", fontweight = 'bold')
        ax.set_xlabel("Angle", fontweight = 'bold')
        ax.set_zlabel("Length       ", fontweight = 'bold')
        ax.set_ylabel("             Stiffness", fontweight='bold')
        ax.yaxis.labelpad=20
        ax.zaxis.labelpad=15
        ax.xaxis.labelpad=9

        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('white')
        ax.yaxis.pane.set_edgecolor('white')
        ax.zaxis.pane.set_edgecolor('white')
        ax.w_xaxis.line.set_color("white")
        ax.w_yaxis.line.set_color("white")
        ax.w_zaxis.line.set_color("white")
        ax.xaxis._axinfo['tick']['color']='w'
        ax.yaxis._axinfo['tick']['color']='w'
        ax.zaxis._axinfo['tick']['color']='w'
        ax.grid(False)

        ##Empty Graph
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.view_init(25, -77)
        
        xpoints = [5,30,60]
        ypoints = [8,32,64]
        zpoints = [50,75,100]
        ymin = ypoints[0]-round((ypoints[2]-ypoints[0])/3)
        xmax = xpoints[2]+round((xpoints[2]-xpoints[0])/5)
        for i in xpoints:
            for j in ypoints:  
                ax.plot([i,i],[j,j],[zpoints[0],zpoints[2]],color = 'k', linewidth = 0.5)
        for i in xpoints:
            for k in zpoints:
                if k == 50:
                    ax.plot([i,i],[ymin,ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([i,i],[ypoints[0],ypoints[2]],[k,k],color = 'k', linewidth = 0.5)
        for j in ypoints:
            for k in zpoints:
                if j == 64 or k == 50:
                    ax.plot([xpoints[0],xmax],[j,j],[k,k],color = 'k', linewidth = 0.5)
                else:
                    ax.plot([xpoints[0],xpoints[2]],[j,j],[k,k],color = 'k', linewidth = 0.5)
        ax.set_xlim(0,60)
        plt.xticks([5,30,60], ['5\u00b0','30\u00b0','60\u00b0'])
        ax.set_zlim(50,100)
        ax.set_zticks([52,77,102])
        ax.set_zticklabels(['   50mm','   75mm','   100mm'])
        ax.set_ylim(0,70)
        plt.yticks([14,38,68], ['   80mJ/r','     320mJ/r','     640mJ/r'])
        ax.set_title("Leg Configuration Key", fontweight = 'bold')
        ax.set_xlabel("Angle", fontweight = 'bold')
        ax.set_zlabel("Length       ", fontweight = 'bold')
        ax.set_ylabel("             Stiffness", fontweight='bold')
        ax.yaxis.labelpad=20
        ax.zaxis.labelpad=15
        ax.xaxis.labelpad=9

        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor('white')
        ax.yaxis.pane.set_edgecolor('white')
        ax.zaxis.pane.set_edgecolor('white')
        ax.w_xaxis.line.set_color("white")
        ax.w_yaxis.line.set_color("white")
        ax.w_zaxis.line.set_color("white")
        ax.xaxis._axinfo['tick']['color']='w'
        ax.yaxis._axinfo['tick']['color']='w'
        ax.zaxis._axinfo['tick']['color']='w'
        ax.grid(False)

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.view_init(25, -77)
        
        # cmap = mpl.cm.copper
        # norm = mpl.colors.Normalize(vmin=0,vmax=1)
        # ax.scatter(xs = angle, ys = stiffness, zs = length, s = sizeBC4, c=avgBC4, cmap = cmap, norm = norm)

        # ax.set_xlim(0,60)
        # ax.set_zlim(50,100)
        # ax.set_ylim(0,70)
        # ax.set_title("Average 4-Leg Body Contact Failure")
        # ax.set_xlabel("Angle (deg)")
        # ax.set_zlabel("Length (mm)")
        # ax.set_ylabel("Stiffness (Nm/rad)")


        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.view_init(25, -77)
        
        # cmap = mpl.cm.Wistia
        # norm = mpl.colors.Normalize(vmin=0,vmax=1)
        # ax.scatter(xs = angle, ys = stiffness, zs = length, s = sizeLeg2, c=avgLeg2, cmap = cmap, norm = norm)

        # ax.set_xlim(0,60)
        # ax.set_zlim(50,100)
        # ax.set_ylim(0,70)
        # ax.set_title("Average 2-Leg Failure")
        # ax.set_xlabel("Angle (deg)")
        # ax.set_zlabel("Length (mm)")
        # ax.set_ylabel("Stiffness (Nm/rad)")

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.view_init(25, -77)
        
        # cmap = mpl.cm.gist_yarg
        # norm = mpl.colors.Normalize(vmin=0,vmax=1)
        # ax.scatter(xs = angle, ys = stiffness, zs = length, s = sizeNoLeg, c=avgNoLeg, cmap = cmap, norm = norm)

        # ax.set_xlim(0,60)
        # ax.set_zlim(50,100)
        # ax.set_ylim(0,70)
        # ax.set_title("Average No Leg Failure")
        # ax.set_xlabel("Angle (deg)")
        # ax.set_zlabel("Length (mm)")
        # ax.set_ylabel("Stiffness (Nm/rad)")


        ##Color and Size Scale
        fig, ax = plt.subplots(figsize=(5,6))
        cmap = mpl.cm.gist_yarg
        norm = mpl.colors.Normalize(vmin=0,vmax=1)
        scale = [0, 0, 0.2, 0.4, 0.6, 0.8, 1.0]
        size = [round(num*700) for num in scale]
        ax.scatter([1, 0, 0, 0, 0, 0, 0], scale, s = size, c = [item*0.8+0.2 for item in scale], cmap = cmap, norm = norm)
        plt.yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0], ["0%", "20%", "40%", "60%", "80%","100%"])
        plt.xticks([])
        ax.spines['top'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)

        if saveFig==True:
            plt.savefig(f'{os.path.join(self.log_dir, filename)}/ScalePlotJet.pdf',dpi=300)

        
        fig = plt.figure(figsize=(14,14))
        origin = np.array([[0],[0]]) # origin point

        count = 0
        for UBm in np.linspace(4,10,4):
            for UBA in np.linspace(45,90,4):
                count += 1
                UB = UBm/10
                LB = UB-0.2
                LBA = UBA-15
                ax = fig.add_subplot(4,4,17-count,projection='polar')
                plt.quiver(*origin, [LBA*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=1/UB)
                plt.quiver(*origin, [UBA*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=1/UB)
                plt.quiver(*origin, [LBA*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=1/LB)
                plt.quiver(*origin, [UBA*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=1/LB)
                theta = np.linspace(LBA*np.pi/180, UBA*np.pi/180, 50)
                r = np.linspace(LB,LB, 50)
                r2 = np.linspace(UB,UB, 50)
                plt.polar(theta, r, color = 'k')
                plt.polar(theta, r2, color = 'k')
                ax.grid(False)
                ax.axis("off")

                Ulimit = np.linspace(1,1, 50)
                Llimit = np.linspace(0.2,0.2, 50)
                thetarange = np.linspace(30*np.pi/180, 90*np.pi/180, 50)
                plt.fill_between(thetarange, Llimit, Ulimit, color = [0.5,0.5,0.5], alpha=0.2)
                plt.fill_between(theta, r, r2, alpha=0.2)

        if saveFig==True:
            plt.savefig(f'{os.path.join(self.log_dir, filename)}/ApproachGraphs.pdf',dpi=300)

        fig = plt.figure(figsize=(4,4))
        ax = fig.add_subplot(111,projection='polar')
        plt.quiver(*origin, [30*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=1/3.5)
        plt.quiver(*origin, [90*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=1/3.5)
        plt.quiver(*origin, [30*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=2)
        plt.quiver(*origin, [90*np.pi/180], [1], color= 'k', angles='xy', scale_units='xy', scale=2)
        Ulimit = np.linspace(3.5,3.5, 50)
        Llimit = np.linspace(0.5,0.5, 50)
        thetarange = np.linspace(30*np.pi/180, 90*np.pi/180, 50)
        plt.fill_between(thetarange, Llimit, Ulimit, color = [0.5,0.5,0.5], alpha=0.2)
        plt.xlim(30*np.pi/180, 90*np.pi/180)
        plt.ylim(0, 3.5)
        ax.set_ylabel("Approach Speed (m/s)")
        ax.set_xticks([30*np.pi/180,45*np.pi/180,60*np.pi/180,75*np.pi/180,90*np.pi/180])
        ax.set_yticks([0.5,1.25,2.0,2.75,3.5])
        ax.set_xlabel("Approach Angle (deg)")
        ax.xaxis.set_label_position('top') 
        ax.xaxis.labelpad=20

        plt.show(block=False)
        plt.pause(0.001) # Pause for interval seconds.
        input("hit[enter] to end.")
        plt.close('all') # all open plots are correctly closed after each run        

        # ## DEFINE INTERPOLATION GRID
        # R_list = np.linspace(R.min(),R.max(),num=50,endpoint=True).reshape(1,-1)
        # Theta_list = np.linspace(Theta.min(),Theta.max(),num=50,endpoint=True).reshape(1,-1)
        # R_grid, Theta_grid = np.meshgrid(R_list, Theta_list)
        
        # ## INTERPOLATE DATA
        # LR_interp = griddata((R, Theta), C, (R_list, Theta_list.T), method='linear')
        # LR_interp += 0.001
        

        # ## INIT PLOT INFO
        # fig = plt.figure(figsize=(6,6))
        # ax = fig.add_subplot(projection='polar')
        # cmap = mpl.cm.jet
        # norm = mpl.colors.Normalize(vmin=0,vmax=1)
        
        # ax.contourf(np.radians(Theta_grid),R_grid,LR_interp,cmap=cmap,norm=norm,levels=30)
        # # ax.scatter(np.radians(Theta),R,c=C,cmap=cmap,norm=norm)
        # # ax.scatter(np.radians(Theta_grid).flatten(),R_grid.flatten(),c=LR_interp.flatten(),cmap=cmap,norm=norm)

        # ax.set_xticks(np.radians(np.arange(-90,90+15,15)))
        # ax.set_thetamin(Theta.min())
        # ax.set_thetamax(Theta.max())

        # ax.set_rticks([0.0,1.0,2.0,3.0,4.0,4.5])
        # ax.set_rmin(0)
        # ax.set_rmax(R.max())
        


        # # ## AXIS LABELS    
        # # ax.text(np.radians(7.5),2,'Flight Velocity (m/s)',
        # #     rotation=18,ha='center',va='center')

        # # ax.text(np.radians(60),4.5,'Flight Angle (deg)',
        # #     rotation=0,ha='left',va='center')

        # if saveFig==True:
        #     plt.savefig(f'{self.TB_log_path}/Landing_Rate_Fig_20Trials.pdf',dpi=300)

        # plt.show()

if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.CrazyflieEnv_DeepRL import CrazyflieEnv_DeepRL
    from Envs.CrazyflieEnv_DeepRL_Tau import CrazyflieEnv_DeepRL_Tau
    from crazyflie_projects.Leg_Design_Analysis.Envs.CrazyflieEnv_DeepRL_LDA import CrazyflieEnv_DeepRL_LDA

    from Envs.CF_Env_2D import CF_Env_2D

    ## INITIATE ENVIRONMENT
    # env = CrazyflieEnv_DeepRL(GZ_Timeout=True,Vel_range=[1.0,3.0],Phi_range=[30,90])
    # log_dir = f"{BASE_PATH}/crazyflie_projects/DeepRL/TB_Logs/{env.env_name}"




    # ## CREATE NEW DEEP RL MODEL 
    # log_name = f"SAC--{current_time}--{env.modelInitials}"    
    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.create_model()
    # PolicyTrainer.train_model()

    # ================================================================= ##
    
    # LOAD DEEP RL MODEL
    # log_name = "SAC--01_21-16:07--NL_0"
    # t_step_load = 8000

    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.load_model(t_step_load)
    # PolicyTrainer.train_model(reset_timesteps=False)
    # PolicyTrainer.test_landing_performance(Vel_range=[2.0,3.0],Phi_range=[60,70])

     # ================================================================= ##

    # # COLLECT LANDING PERFORMANCE DATA
    # env = CrazyflieEnv_DeepRL_LDA(GZ_Timeout=True,Vel_range=[0.5,3.5],Phi_range=[30,90])
    # log_dir = f"{BASE_PATH}/crazyflie_projects/Leg_Design_Analysis/TB_Logs/{env.env_name}"
    # log_name = "SAC--01_25-20:21--LDA_A60_L100_K64_0"
    # t_step_load = 91000

    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.load_model(t_step_load)
    # PolicyTrainer.test_landing_performance()

    # ================================================================= ##

    # # PLOT LANDING PERFORMANCE
    # env = None
    # log_dir = f"{BASE_PATH}/crazyflie_projects/DeepRL/TB_Logs/CF_Gazebo"
    # log_name = "A05_L75_K32_0--Deg_90--SAC_02_05-12:32_0"
    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.Plot_Landing_Performance(saveFig=True)

