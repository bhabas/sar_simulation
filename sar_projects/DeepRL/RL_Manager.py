## STANDARD IMPORTS
import os
from datetime import datetime,timedelta
import numpy as np
import pandas as pd
import torch as th
import yaml
import pandas as pd
import csv
import time 
import rospy
import rospkg
import glob
import boto3

import matplotlib.pyplot as plt
import matplotlib as mpl
from scipy.interpolate import griddata

## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common import utils
from stable_baselines3.common.env_util import make_vec_env




## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
current_datetime = datetime.now()
current_time = current_datetime.strftime("%m_%d-%H:%M")

class RL_Training_Manager():
    def __init__(self,env,Log_dir,Log_name,env_kwargs=None):

        self.vec_env = make_vec_env(env, env_kwargs=env_kwargs)
        self.env = self.vec_env.envs[0].unwrapped

        self.Log_name = Log_name
        self.Log_dir = Log_dir

        self.Log_subdir = os.path.join(Log_dir, Log_name)
        self.Model_subdir = os.path.join(self.Log_subdir, "Models")
        self.TB_log_subdir = os.path.join(self.Log_subdir, "TB_Logs")

        os.makedirs(self.Log_subdir, exist_ok=True)
        os.makedirs(self.Model_subdir, exist_ok=True)
        os.makedirs(self.TB_log_subdir, exist_ok=True)

    def create_model(self,model_kwargs,write_config=True):

        self.model = SAC(
            "MlpPolicy",
            env=self.vec_env,
            gamma=model_kwargs["gamma"],
            learning_rate=model_kwargs["learning_rate"],
            ent_coef=model_kwargs["ent_coef"],
            target_entropy=model_kwargs["target_entropy"],
            buffer_size=model_kwargs["buffer_size"],
            policy_kwargs=dict(activation_fn=th.nn.LeakyReLU,net_arch=model_kwargs["net_arch"]),
            replay_buffer_kwargs=dict(handle_timeout_termination=False),
            learning_starts=0,
            verbose=1,
            device='cpu',
            tensorboard_log=self.TB_log_subdir
        )

        if write_config:
            self.write_config_file()
        
    def load_model(self,t_step: int, Log_name: str, Params_only=False, load_replay_buffer=True):
        
        ## SEARCH FOR BOTH MODEL AND REPLAY BUFFER WITH WILDCARD FOR OPTIONAL SUFFIX
        model_pattern = f"model_{int(t_step)}_steps*.zip" 
        replay_buffer_pattern = f"replay_buffer_{int(t_step)}_steps*.pkl" 

        ## FIND MODEL AND REPLAY BUFFER FILES
        model_files = glob.glob(os.path.join(self.Log_dir, Log_name, "Models", model_pattern))
        replay_buffer_files = glob.glob(os.path.join(self.Log_dir, Log_name, "Models", replay_buffer_pattern))

        ## LOAD MODEL AND REPLAY BUFFER
        if Params_only:
            print(f"Loading Model params...")
            model_path = model_files[0]  # Taking the first match
            loaded_model = SAC.load(
                model_path,
                device='cpu',
            )
            self.model.buffer_size = int(20e3)
            self.model.policy.load_state_dict(loaded_model.policy.state_dict())
        
        else:
            print(f"Loading Model...")
            model_path = model_files[0]  # Taking the first match
            self.model = SAC.load(
                model_path,
                env=self.vec_env,
                device='cpu',
            )

        if load_replay_buffer:

            print(f"Loading Replay Buffer...")
            replay_buffer_path = replay_buffer_files[0]  # Similarly, taking the first match
            self.model.load_replay_buffer(replay_buffer_path)


    def train_model(self,save_freq=25e3,reset_timesteps=False,total_timesteps=500e3):

        if reset_timesteps == True:

            self.model.tensorboard_log = self.TB_log_subdir
            self.model.learning_starts = 0



        reward_callback = RewardCallback(self,save_freq=save_freq,)

        self.model.learn(
            total_timesteps=int(total_timesteps),
            callback=reward_callback,
            tb_log_name="TB_Log",
            reset_num_timesteps=reset_timesteps,
        )

    def test_policy(self,V_mag=None,V_angle=None,Plane_Angle=None):

        if V_mag != None:
            self.env.V_mag_range = [V_mag,V_mag]

        if V_angle != None:
            self.env.V_angle_range = [V_angle,V_angle]

        if Plane_Angle != None:
            self.env.Plane_Angle_range = [Plane_Angle,Plane_Angle]

        obs,_ = self.env.reset()
        terminated = False
        truncated = False
 
        while not (terminated or truncated):
            action,_ = self.model.predict(obs)
            obs,reward,terminated,truncated,_ = self.env.step(action)

        return obs,reward
    
    def sweep_policy(self,Plane_Angle_Step=45,V_mag_Step=0.5,V_angle_Step=10,n=1):

        Plane_Angle_arr = np.arange(self.env.Plane_Angle_range[0], self.env.Plane_Angle_range[1] + Plane_Angle_Step, Plane_Angle_Step)
        V_mag_arr = np.arange(self.env.V_mag_range[0], self.env.V_mag_range[1] + V_mag_Step, V_mag_Step)
        V_angle_arr = np.arange(self.env.V_angle_range[0], self.env.V_angle_range[1] + V_angle_Step, V_angle_Step)
        
        for Plane_Angle in Plane_Angle_arr:
            for V_mag in V_mag_arr:
                for V_angle in V_angle_arr:
                    for _ in range(n):

                        obs,reward = self.test_policy(V_mag,V_angle,Plane_Angle)
                        print(f"Plane_Angle: {Plane_Angle:.2f}  V_mag: {V_mag:.2f}  V_angle: {V_angle:.2f}")
                        print(f"Reward: {reward:.3f} \t Tau_CR: {obs[0]:.2f}  Theta_x: {obs[1]:.2f}  D_perp: {obs[2]:.2f}\n\n")

    def collect_landing_performance(self,fileName=None,Plane_Angle_range=[0,0,1],V_angle_range=[30,90,7],V_mag_range=[1.0,4.0,7],n_trials=1):

        if fileName is None:
            fileName = "PolicyPerformance_Data.csv"
        filePath = os.path.join(self.Log_subdir,fileName)
         
        ## GENERATE SWEEP ARRAYS
        Plane_Angle_arr = np.arange(Plane_Angle_range[0],Plane_Angle_range[1] + Plane_Angle_range[2],Plane_Angle_range[2])
        V_mag_arr = np.arange(V_mag_range[0],V_mag_range[1] + V_mag_range[2],V_mag_range[2])
        V_angle_arr = np.arange(V_angle_range[0],V_angle_range[1] + V_angle_range[2],V_angle_range[2])


        def EMA(cur_val,prev_val,alpha = 0.15):            
            return alpha*cur_val + (1-alpha)*(prev_val)
        
        def round_list_elements(lst, precision=3):
            """Round all numerical elements in a list to a specified precision."""
            rounded_list = []
            for item in lst:
                if isinstance(item, np.floating):  # Check if the item is a float
                    rounded_list.append(round(item, precision))
                else:
                    rounded_list.append(item)  # Leave non-float items unchanged
            return rounded_list

        ## TIME ESTIMATION FILTER INITIALIZATION
        num_trials = len(V_mag_arr)*len(V_angle_arr)*len(Plane_Angle_arr)*n_trials
        idx = 0
        t_delta = 0
        t_delta_prev = 0
        t_init = time.time()

        if not os.path.exists(filePath):
            with open(filePath,'w') as file:
                writer = csv.writer(file,delimiter=',')
                writer.writerow([
                    "V_mag", "V_angle", "Plane_Angle", "Trial_num",

                    "--",

                    "Pad_Connections",
                    "BodyContact","ForelegContact","HindlegContact",

                    "--",
                    
                    "a_Trg_trg",
                    "a_Rot_trg",
                    "Vel_mag_B_O_trg","Vel_angle_B_O_trg",
                    "Vel_mag_B_P_trg","Vel_angle_B_P_trg",

                    "Tau_CR_trg",
                    "Tau_trg",
                    "Theta_x_trg",
                    "D_perp_CR_trg",
                    "D_perp_trg",

                    "--",

                    "Phi_B_O_impact",
                    "Phi_B_P_impact",
                    "Omega_B_O_impact",

                    "Vel_B_P_impact_x","Vel_B_P_impact_z",
                    
                    "Impact_Magnitude",
                    "Force_Impact_x","Force_Impact_y","Force_Impact_z",

                    "--",

                    "reward","reward_vals",
                    "NN_Output_trg","a_Rot_scale",

                    "--",

                    "4_Leg_NBC","4_Leg_BC",
                    "2_Leg_NBC","2_Leg_BC",
                    "0_Leg_NBC","0_Leg_BC",
                ])

        for Plane_Angle in Plane_Angle_arr:
            for V_mag in V_mag_arr:
                for V_angle in V_angle_arr:
                    for trial in range(n_trials):

                        ## CONVERT RELATIVE ANGLES TO GLOBAL ANGLE
                        A1 = V_angle - Plane_Angle

                        ## ANGLE CAPS TO ENSURE +X DIRECTION
                        B1 = -90
                        B2 = 90

                        if A1 < B1 or A1 > B2:
                            continue

                        t_trial_start = time.time()

                        ## TEST POLICY FOR GIVEN FLIGHT CONDITIONS
                        self.test_policy(V_mag,V_angle,Plane_Angle)

                        if self.env.Policy_Type == "DEEP_RL_SB3":
                            NN_Output_trg = self.policy_output(self.env.obs_trg)
                            a_Trg_trg = self.env.action_trg[0]

                        else:
                            NN_Output_trg = self.env.NN_Output_trg
                            a_Trg_trg = self.env.a_Trg_trg

                        PC = self.env.Pad_Connections
                        BC = self.env.BodyContact_Flag

                        if PC >= 3 and BC == False:       # 4_Leg_NBC
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

                        ## APPEND RECORDED VALUES TO CSV FILE
                        with open(filePath,'a') as file:
                            writer = csv.writer(file,delimiter=',',quoting=csv.QUOTE_NONE,escapechar='\\')

                            row_data = [
                            
                                V_mag,V_angle,Plane_Angle,trial,
                                "--",
                                self.env.Pad_Connections,
                                self.env.BodyContact_Flag,self.env.ForelegContact_Flag,self.env.HindlegContact_Flag,
                                "--",
                                np.round(a_Trg_trg,3),self.env.a_Rot_trg,
                                self.env.Vel_mag_B_O_trg,self.env.Vel_angle_B_O_trg,
                                self.env.Vel_mag_B_P_trg,self.env.Vel_angle_B_P_trg,
                                self.env.Tau_CR_trg,
                                self.env.Tau_trg,
                                self.env.Theta_x_trg,
                                self.env.D_perp_CR_trg,
                                self.env.D_perp_trg,
                                "--",
                                self.env.Eul_B_O_impact_Ext[1],
                                self.env.Eul_P_B_impact_Ext[1],
                                self.env.Omega_B_P_impact_Ext[1],
                                self.env.V_B_P_impact_Ext[0],self.env.V_B_P_impact_Ext[2],
                                self.env.Impact_Magnitude,
                                self.env.Force_impact_x,self.env.Force_impact_y,self.env.Force_impact_z,
                                "--",
                                np.round(self.env.reward,3),np.round(self.env.reward_vals,3),
                                np.round(NN_Output_trg,3),np.round(self.env.Ang_Acc_range,0),
                                "--",
                                LS[0],LS[1],
                                LS[2],LS[3],
                                LS[4],LS[5],

                            ]

                            rounded_row_data = round_list_elements(row_data)
                            writer.writerow(rounded_row_data)



                            ## CALCULATE AVERAGE TIME PER EPISODE
                            t_now = time.time() - t_init
                            t_delta = time.time() - t_trial_start
                            t_delta_avg = EMA(t_delta,t_delta_prev,alpha=0.05)
                            t_delta_prev = t_delta_avg
                            idx += 1

                            TTC = np.round(t_delta_avg*(num_trials-idx)) # Time to completion
                            t_now = np.round(t_now)
                            print(f"Flight Conditions: ({V_mag:.02f} m/s,{V_angle:.02f} deg, {Plane_Angle:.02f} deg) Index: {idx}/{num_trials} \tPercent: {100*idx/num_trials:.2f}% \tTTC: {str(timedelta(seconds=TTC))} \tElapsed: {str(timedelta(seconds=t_now))}")

    def plot_landing_performance(self,PlaneAngle=0,fileName=None,saveFig=False,showFig=True):

        if fileName == None:
            fileName = "PolicyPerformance_Data.csv"
        filePath = os.path.join(self.Log_subdir,fileName)

        ## READ CSV FILE
        df = pd.read_csv(filePath,sep=',',comment="#")

        cols_to_keep = [col for col in df.columns if not col.startswith('--')]
        df = df[cols_to_keep]
        df.drop(["reward_vals","NN_Output_trg","a_Rot_scale"],axis=1,inplace=True)

        df2 = df.groupby(["V_mag","V_angle","Plane_Angle"]).mean().round(3).reset_index()
        df2.query(f"Plane_Angle == {PlaneAngle}",inplace=True)
        
        
        ## COLLECT DATA
        R = df2.iloc[:]['V_mag']
        Theta = df2.iloc[:]['V_angle']-PlaneAngle
        C = df2.iloc[:]['4_Leg_NBC']



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

        # ax.set_xticks(np.radians(np.arange(-90,90+15,15)))
        ax.set_thetamin(0-PlaneAngle)
        ax.set_thetamax(180-PlaneAngle)

        ax.set_rticks([0.0,1.0,2.0,3.0,4.0,4.5])
        ax.set_rmin(0)
        ax.set_rmax(R.max())
        


        # ## AXIS LABELS    
        # ax.text(np.radians(7.5),2,'Flight Velocity (m/s)',
        #     rotation=18,ha='center',va='center')

        # ax.text(np.radians(60),4.5,'Flight Angle (deg)',
        #     rotation=0,ha='left',va='center')

        if saveFig==True:
            plt.savefig(f'{self.Log_subdir}/Landing_Rate_Fig_PlaneAngle_{PlaneAngle:.0f}.pdf',dpi=300)

        if showFig==True:
            plt.show(block=True)
        
    def save_NN_to_C_header(self):

        FileName = f"NN_Params_DeepRL.h"
        f = open(os.path.join(self.Log_subdir,FileName),'a')
        f.truncate(0) ## Clears contents of file

        f.write(f"// Model: {self.Log_name} \t SAR_Type: {self.env.SAR_Type} \t SAR_Config: {self.env.SAR_Config}\n")
        f.write("static char NN_Params_DeepRL[] = {\n")

        num_hidden_layers = np.array([3]).reshape(-1,1)

        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,num_hidden_layers,
                    fmt='"%.0f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{num_hidden_layers.shape[0]},"\t"{num_hidden_layers.shape[1]},"',
                    footer='"*"\n')
        
        ## SCALER ARRAY DIMENSIONS
        obs_len = self.model.observation_space.shape[0]
        scaling_means = np.zeros(obs_len).reshape(-1,1)
        scaling_std = np.ones(obs_len).reshape(-1,1)

        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaling_means,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaling_means.shape[0]},"\t"{scaling_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaling_std,
                    fmt='"%.5f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaling_std.shape[0]},"\t"{scaling_std.shape[1]},"',
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

    def write_config_file(self):
        config_path = os.path.join(self.Log_subdir,"Config.yaml")

        General_Dict = dict(

            SAR_SETTINGS = dict(
                SAR_Type = self.env.SAR_Type,
                SAR_Config = self.env.SAR_Config,
            ),

            PLANE_SETTINGS = dict(
                Plane_Type = self.env.Plane_Type,
                Plane_Config = self.env.Plane_Config,
            ),

            ENV_SETTINGS = dict(
                Env_Name = self.env.Env_Name,
                V_mag_Limts = self.env.V_mag_range,
                V_angle_Limits = self.env.V_angle_range,
                Plane_Angle_Limits = self.env.Plane_Angle_range,
                Ang_Acc_Limits = self.env.Ang_Acc_range,
            ),

            MODEL_SETTINGS = dict(
                Mass_Std = self.env.Mass_std,
                Iyy_Std = self.env.Iyy_std,
                Ref_Mass = self.env.Ref_Mass,
                Ref_Ixx = self.env.Ref_Ixx,
                Ref_Iyy = self.env.Ref_Iyy,
                Ref_Izz = self.env.Ref_Izz,
                L_eff = self.env.L_eff,
                Gamma_eff = float(self.env.Gamma_eff),
                Forward_Reach = self.env.Forward_Reach,
            ),

            LEARNING_MODEL = dict(
                Policy = self.model.policy.__class__.__name__,
                Observation_Layer = self.model.policy.observation_space.shape[0],
                Network_Layers = self.model.policy.net_arch,
                Action_Layer = self.model.policy.action_space.shape[0]*2,
                Action_Space_High = self.model.policy.action_space.high.tolist(),
                Action_Space_Low = self.model.policy.action_space.low.tolist(),
                Gamma = self.model.gamma,
                Learning_Rate = self.model.learning_rate,
            ),

            REWARD_SETTINGS=self.env.reward_weights
        )

        with open(config_path, 'w') as outfile:
            yaml.dump(General_Dict,outfile,default_flow_style=False,sort_keys=False)

    def load_config_file(self,config_path):

        with open(config_path, 'r') as file:
            config_dict = yaml.safe_load(file)

        self.env.setAngAcc_range(config_dict['ENV_SETTINGS']['Ang_Acc_Limits'])

    def policy_output(self,obs):
        
        ## CONVERT OBS TO TENSOR
        obs = th.FloatTensor([obs])

        ## PASS OBS THROUGH NN
        actor = self.model.policy.actor
        latent_pi = actor.latent_pi(obs)
        mean_actions = actor.mu(latent_pi)
        log_std = actor.log_std(latent_pi)

        # CLAMP THE LOG STANDARD DEVIATION OF THE ACTOR (FOR STABILITY)
        LOG_STD_MAX = 2
        LOG_STD_MIN = -20
        log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)

        ## CONVERT LOG_STD TO STD
        action_log_std = log_std
        action_log_std = action_log_std.detach().numpy()[0]
        action_std = np.exp(action_log_std)
        # squished_action_std = np.tanh(action_std) ## THIS OPERATION IS INVALID (TANH IS NON-LINEAR)

        ## GRAB ACTION DISTRIBUTION MEAN
        action_mean = mean_actions
        action_mean = action_mean.detach().numpy()[0]
        squished_action_mean = np.tanh(action_mean) ## MEAN POSITION SCALES APPROPRIATELY THOUGH

        return np.hstack((squished_action_mean,action_std))

    def upload_file_to_S3(self,file_path,bucket_name,object_name=None):
        """
        Upload a file to an S3 bucket

        :param file_path: Full path to file to upload
        :param bucket_name: Bucket to upload to
        :param object_name: S3 object name. If not specified, just the file name is used
        """
        # Extract the file name from the file path if object_name is not specified
        if object_name is None:
            object_name = os.path.basename(file_path)

        # Upload the file
        s3_client = boto3.client('s3')
        try:
            response = s3_client.upload_file(file_path, bucket_name, object_name)

        except Exception as e:
            print(f"Upload failed: {e}")
        else:
            print(f"File {file_path} uploaded to {bucket_name}/{object_name}")



class RewardCallback(BaseCallback):
    def __init__(self, RLM, 
                 check_freq: int = 500, 
                 save_freq: int = 25_000, 
                 keep_last_n_models: int = 5,
                 verbose=0):
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_freq = save_freq
        self.best_mean_reward = -np.inf
        self.RLM = RLM
        self.s3_client = boto3.client('s3')


        self.n_prev_episodes = 25
        self.episode_rewards = []
        self.saved_models = []
        self.keep_last_n_models = keep_last_n_models


    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        self.env = self.training_env.envs[0].unwrapped
        TB_Log_pattern = f"TB_Log_*" 
        TB_log_files = glob.glob(os.path.join(self.RLM.TB_log_subdir, TB_Log_pattern))
        self.TB_Log_path = TB_log_files[0]        
        

    def _on_step(self) -> bool:

        ## CHECK FOR MODEL PERFORMANCE AND SAVE IF IMPROVED
        if self.num_timesteps % self.check_freq == 0:

            ## UPLOAD TB LOG TO SB3
            # self.RLM.upload_file_to_S3(self.TB_Log_path,"robotlandingproject--deeprl--logs",object_name=os.path.join(self.RLM.log_name,self.TB_Log))


            ## COMPUTE THE MEAN REWARD FOR THE LAST 'CHECK_FREQ' EPISODES
            mean_reward = np.mean(self.episode_rewards)
            if mean_reward > self.best_mean_reward:
                self.best_mean_reward = mean_reward

                ## SAVE MODEL AND REPLAY BUFFER
                model_path = os.path.join(self.RLM.Model_subdir, f"model_{self.num_timesteps}_steps_BestModel")
                replay_buffer_path = os.path.join(self.RLM.Model_subdir, f"replay_buffer_{self.num_timesteps}_steps_BestModel")

                self.model.save(model_path)
                self.model.save_replay_buffer(replay_buffer_path)

                if self.verbose > 0:
                    print(f"New best mean reward: {self.best_mean_reward:.2f} - Model and replay buffer saved.")

                ## TRACK SAVED MODELS FOR DELETION
                self.saved_models.append((model_path, replay_buffer_path))
                self._keep_last_n_models()

        ## SAVE MODEL EVERY N TIMESTEPS
        if self.num_timesteps % self.save_freq == 0:

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            model_path = os.path.join(self.RLM.Model_subdir, f"model_{self.num_timesteps}_steps")
            replay_buffer_path = os.path.join(self.RLM.Model_subdir, f"replay_buffer_{self.num_timesteps}_steps")

            self.model.save(model_path)
            self.model.save_replay_buffer(replay_buffer_path)

        ## CHECK IF EPISODE IS DONE
        if self.locals["dones"].item():

            episode_reward = self.locals["rewards"]
            self.episode_rewards.append(episode_reward.item())
            if len(self.episode_rewards) > self.n_prev_episodes:
                self.episode_rewards.pop(0)  # Remove the oldest reward

            ## TB LOGGING VALUES
            info_dict = self.locals["infos"][0]
            self.logger.record('Custom/K_ep',self.env.K_ep)
            self.logger.record('Custom/Reward',episode_reward.item())
            self.logger.record('Custom/TestCondition_idx',info_dict["TestCondition_idx"])

            self.logger.record('z_Custom/Vel_mag',info_dict["V_mag"])
            self.logger.record('z_Custom/Vel_angle',info_dict["V_angle"])
            self.logger.record('z_Custom/Plane_Angle',info_dict["Plane_Angle"])
            self.logger.record('z_Custom/a_Rot_trg',info_dict["a_Rot"])
            self.logger.record('z_Custom/Tau_CR_trg',info_dict["Tau_CR_trg"])
            self.logger.record('z_Custom/Trg_Flag',int(info_dict["Trg_Flag"]))
            self.logger.record('z_Custom/Impact_Flag_Ext',int(info_dict["Impact_Flag_Ext"]))
            self.logger.record('z_Custom/D_perp_pad_min',info_dict["D_perp_pad_min"])
            self.logger.record('z_Rewards_Components/R_Dist',info_dict["reward_vals"][0])
            self.logger.record('z_Rewards_Components/R_tau',info_dict["reward_vals"][1])
            self.logger.record('z_Rewards_Components/R_tx',info_dict["reward_vals"][2])
            self.logger.record('z_Rewards_Components/R_LT',info_dict["reward_vals"][3])
            self.logger.record('z_Rewards_Components/R_GM',info_dict["reward_vals"][4])
            self.logger.record('z_Rewards_Components/R_phi',info_dict["reward_vals"][5])
            self.logger.record('z_Rewards_Components/R_Legs',info_dict["reward_vals"][6])

        

        return True
    
    def _keep_last_n_models(self):

        ## REMOVE OLDER TOP MODELS AND REPLAY BUFFERS
        if len(self.saved_models) > self.keep_last_n_models:

            for model_path, replay_buffer_path in self.saved_models[:-self.keep_last_n_models]:
                if os.path.exists(model_path + ".zip"):
                    os.remove(model_path + ".zip")
                if os.path.exists(replay_buffer_path + ".pkl"):
                    os.remove(replay_buffer_path + ".pkl")

            self.saved_models = self.saved_models[-self.keep_last_n_models:]
    
    def _on_rollout_start(self) -> None:
        """
        A rollout is the collection of environment interaction
        using the current policy.
        This event is triggered before collecting new samples.
        """
        pass
    
    def _on_rollout_end(self) -> None:
        """
        This event is triggered before updating the policy.
        """
        pass



if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.SAR_2D_DeepRL import SAR_2D_Env


    current_datetime = datetime.now()
    current_time = current_datetime.strftime("%m-%d--%H:%M:%S")
    log_name = f"DeepRL_Policy_{current_time}"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 0],
        "V_mag_range": [1.5,4.0],
        "V_angle_range": [10,90],
        "Plane_Angle_range": [0,0],
        "Render": False,
        "GZ_Timeout": False,
    }


    


    


    # log_name = "DeepRL_Policy_03-26--21:50:22"
    # model_dir = f"/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/{log_name}/Models"
    # RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    # RL_Manager.load_model(model_dir,t_step=35000)
    # RL_Manager.sweep_policy(Plane_Angle_range=[0,180,45],V_mag_range=[1.5,4.0,1.0],V_angle_range=[10,170,20],n=1)


    # RL_Manager.collect_landing_performance(
    #     fileName="PolicyPerformance_Data.csv",
    #     Plane_Angle_range=[0,180,45],
    #     V_mag_range=[1.0,4.0,0.5],
    #     V_angle_range=[10,170,5],
    #     n_trials=5
    #     )
    # RL_Manager.plot_landing_performance(PlaneAngle=180)
    
