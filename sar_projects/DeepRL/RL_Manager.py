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
    def __init__(self,env,log_dir,log_name,env_kwargs=None):

        self.vec_env = make_vec_env(env, env_kwargs=env_kwargs)
        self.env = self.vec_env.envs[0].unwrapped

        self.log_name = log_name
        self.log_dir = os.path.join(log_dir, log_name)
        self.model_dir = os.path.join(log_dir, log_name, "Models")

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(self.model_dir, exist_ok=True)

    def create_model(self,gamma=0.999,learning_rate=0.002,net_arch=[12,12,12]):

        self.model = SAC(
            "MlpPolicy",
            env=self.vec_env,
            gamma=gamma,
            learning_rate=learning_rate,
            ent_coef='auto',
            policy_kwargs=dict(activation_fn=th.nn.LeakyReLU,net_arch=dict(pi=net_arch, qf=[256,256,256])),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_dir
        )

        self.write_config_file()
        
    def load_model(self,model_dir,t_step: int):
        
        ## LOAD CONFIG FILE
        config_path = os.path.join(os.path.dirname(model_dir),"Config.yaml")
        self.load_config_file(config_path)
        
        ## LOAD MODEL AND REPLAY BUFFER
        model_str = f"model_{int(t_step)}_steps"
        model_path = os.path.join(model_dir, model_str)

        replay_buffer_str = f"replay_buffer_{int(t_step)}_steps"
        replay_buffer_path = os.path.join(model_dir,replay_buffer_str)

        print(f"Loading Model: {model_str}...")
        self.model = SAC.load(
            model_path,
            env=self.vec_env,
            device='cpu',
            tensorboard_log=self.log_dir,
        )
        self.model.load_replay_buffer(replay_buffer_path)
        print("Model Loaded Successfully!\n")

    def train_model(self,check_freq=10,save_freq=2e3,reset_timesteps=True,total_timesteps=2e6):

        reward_callback = RewardCallback(check_freq=check_freq,save_freq=save_freq,model_dir=self.model_dir)

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
    
    def sweep_policy(self,Plane_Angle_range=[180,180,4],V_angle_range=[-45,-135,4],V_mag_range=[1.0,2.0,4],n=1):
        
        for Plane_Angle in np.linspace(Plane_Angle_range[0],Plane_Angle_range[1],Plane_Angle_range[2]):
            for V_mag in np.linspace(V_mag_range[0],V_mag_range[1],V_mag_range[2]):
                for V_angle in np.linspace(V_angle_range[0],V_angle_range[1],V_angle_range[2]):
                    for _ in range(n):

                        obs,reward = self.test_policy(V_mag,V_angle,Plane_Angle)
                        print(f"Plane_Angle: {Plane_Angle:.2f}  V_mag: {V_mag:.2f}  V_angle: {V_angle:.2f}")
                        print(f"Reward: {reward:.3f} \t Tau_CR: {obs[0]:.2f}  Theta_x: {obs[1]:.2f}  D_perp: {obs[2]:.2f}\n\n")

    def collect_landing_performance(self,fileName=None,V_mag_step=0.25,V_angle_step=5,Plane_Angle_step=45,n_episodes=5):

        if fileName is None:
            fileName = "PolicyPerformance_Data.csv"
        filePath = os.path.join(self.log_dir,fileName)
         

        V_mag_arr = np.arange(self.env.V_mag_range[0],self.env.V_mag_range[1]+V_mag_step,V_mag_step)
        V_angle_arr = np.arange(self.env.V_angle_range[0],self.env.V_angle_range[1]+V_angle_step,V_angle_step)
        Plane_Angle_arr = np.arange(self.env.Plane_Angle_range[0],self.env.Plane_Angle_range[1]+Plane_Angle_step,Plane_Angle_step)

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
        num_trials = len(V_mag_arr)*len(V_angle_arr)*len(Plane_Angle_arr)*n_episodes
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
                    "NN_Output_trg","a_Rot_scale"
                ])

        for Plane_Angle in Plane_Angle_arr:
            for V_angle in V_angle_arr:
                for V_mag in V_mag_arr:
                    for trial in range(n_episodes):

                        t_trial_start = time.time()

                        ## TEST POLICY FOR GIVEN FLIGHT CONDITIONS
                        self.test_policy(V_mag,V_angle,Plane_Angle)

                        if self.env.Policy_Type == "DEEP_RL_SB3":
                            NN_Output_trg = self.policy_output(self.env.obs_trg)
                            a_Trg_trg = self.env.action_trg[0]

                        else:
                            NN_Output_trg = self.env.NN_Output_trg
                            a_Trg_trg = self.env.a_Trg_trg

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
                                np.round(NN_Output_trg,3),np.round(self.env.Ang_Acc_range,0)
                            ]

                            rounded_row_data = round_list_elements(row_data)
                            writer.writerow(rounded_row_data)



                            ## CALCULATE AVERAGE TIME PER EPISODE
                            t_now = time.time() - t_init
                            t_delta = time.time() - t_trial_start
                            t_delta_avg = EMA(t_delta,t_delta_prev,alpha=0.15)
                            t_delta_prev = t_delta_avg
                            idx += 1

                            TTC = np.round(t_delta_avg*(num_trials-idx)) # Time to completion
                            t_now = np.round(t_now)
                            print(f"Flight Conditions: ({V_mag:.02f} m/s,{V_angle:.02f} deg, {Plane_Angle:.02f} deg)\t Index: {idx}/{num_trials} \t Percentage: {100*idx/num_trials:.2f}% \t TTC: {str(timedelta(seconds=TTC))} \t Time Elapsed: {str(timedelta(seconds=t_now))}")

    def save_NN_to_C_header(self):

        FileName = f"NN_Params_DeepRL.h"
        f = open(os.path.join(self.log_dir,FileName),'a')
        f.truncate(0) ## Clears contents of file

        f.write(f"// Model: {self.log_name}\n")
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
        config_path = os.path.join(self.log_dir,"Config.yaml")

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

        # self.env.V_mag_range = config_dict['ENV_SETTINGS']['V_mag_Limts']
        # self.env.V_angle_range = config_dict['ENV_SETTINGS']['V_angle_Limits']
        # self.env.Plane_Angle_range = config_dict['ENV_SETTINGS']['Plane_Angle_Limits']
        self.env.setAngAcc_range(config_dict['ENV_SETTINGS']['Ang_Acc_Limits'])

        self.env.reward_weights = config_dict['REWARD_SETTINGS']

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




class RewardCallback(BaseCallback):
    def __init__(self, check_freq: int, save_freq: int, model_dir: str, verbose=1):
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_freq = save_freq
        self.model_dir = model_dir

        self.reward_avg = [0]
        self.reward_std = [0]

        self.episode_rewards = []
        self.highest_reward = -np.inf

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        self.env = self.training_env.envs[0].unwrapped

    def _on_step(self) -> bool:

        ## SAVE MODEL EVERY N TIMESTEPS
        if self.n_calls % self.save_freq == 0:

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            model_path = os.path.join(self.model_dir, f"model_{self.num_timesteps}_steps")
            replay_buffer_path = os.path.join(self.model_dir, f"replay_buffer_{self.num_timesteps}_steps")

            self.model.save(model_path)
            self.model.save_replay_buffer(replay_buffer_path)

        ## 
        if self.locals["dones"].item():

            episode_reward = self.locals["rewards"]
            self.episode_rewards.append(episode_reward.item())

            ## TB LOGGING VALUES
            self.logger.record('Custom/K_ep',self.env.K_ep)
            self.logger.record('Custom/Reward',episode_reward.item())

            self.logger.record('z_Custom/Vel_mag',self.env.V_mag)
            self.logger.record('z_Custom/Vel_angle',self.env.V_angle)
            self.logger.record('z_Custom/Plane_Angle',self.env.Plane_Angle_deg)
            self.logger.record('z_Custom/a_Trg',self.env.a_Trg_trg)
            self.logger.record('z_Custom/a_Rot',self.env.a_Rot_trg)
            self.logger.record('z_Custom/Flip_Flag',int(self.env.Trg_Flag))
            self.logger.record('z_Custom/Impact_Flag_Ext',int(self.env.Impact_Flag_Ext))
            
            self.logger.record('z_Rewards_Components/R_Dist',self.env.reward_vals[0])
            self.logger.record('z_Rewards_Components/R_tau',self.env.reward_vals[1])
            self.logger.record('z_Rewards_Components/R_LT',self.env.reward_vals[2])
            self.logger.record('z_Rewards_Components/R_GM',self.env.reward_vals[3])
            self.logger.record('z_Rewards_Components/R_phi',self.env.reward_vals[4])
            self.logger.record('z_Rewards_Components/R_Legs',self.env.reward_vals[5])

        return True
    
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
        "V_mag_range": [1.0, 4.0],
        "V_angle_range": [5,90],
        "Plane_Angle_range": [0, 0],
        "Render": False,
        "GZ_Timeout": False,
    }


    # log_name = "DeepRL_Policy_03-08--17:35:23"
    # model_dir = f"/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/{log_name}/Models"
    
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    RL_Manager.create_model(net_arch=[14,14,14])
    RL_Manager.train_model()
    # RL_Manager.load_model(model_dir,t_step=166000)
    # RL_Manager.sweep_policy(Plane_Angle_range=[0,0,1],V_angle_range=[30,90,7],V_mag_range=[1.0,4.0,7],n=1)
    # RL_Manager.collect_landing_performance(
    #     fileName="PolicyPerformance_Data.csv",
    #     V_mag_step=0.5,
    #     V_angle_step=5,
    #     Plane_Angle_step=45,
    #     n_episodes=100
    #     )
    
