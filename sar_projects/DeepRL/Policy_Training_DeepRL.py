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

## PLOTTING IMPORTS
import matplotlib.pyplot as plt
import matplotlib as mpl
import plotly.graph_objects as go
from scipy.interpolate import griddata


## SB3 Imports
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
from stable_baselines3.common import utils

## DEFINE BASE PATH
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('sar_env'))

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%m_%d-%H:%M")

class RewardCallback(BaseCallback):
    def __init__(self, check_freq: int, save_freq: int, model_dir: str, verbose=1):
        """ Callback which monitors training progress and saves the model every [save_freq] timesteps
        and checks for highest mean reward every [check_freq] episodes.

        Args:
            check_freq (int): Number of episodes to average rewards over
            save_freq (int): Number of timesteps to save model
            model_dir (str): Model/replay_buffer save directory
            verbose (int, optional): Verbose. Defaults to 1.
        """        
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_freq = save_freq
        self.model_dir = model_dir

        self.reward_avg = [0]
        self.reward_std = [0]

        self.episode_rewards = []
        self.highest_reward = -np.inf

    def _on_step(self) -> bool:

        ## SAVE MODEL EVERY N TIMESTEPS
        if self.num_timesteps % self.save_freq == 0 and self.num_timesteps >= 500:

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            model = os.path.join(self.model_dir, f"{self.num_timesteps}_step_model")
            replay_buff = os.path.join(self.model_dir, f"{self.num_timesteps}_step_replay_buff")

            self.model.save(model)
            self.model.save_replay_buffer(replay_buff)

        ## CALC MEAN/STD OF REWARDS EVERY N EPISODES
        if self.locals["dones"].item():
            
            episode_reward = self.locals["rewards"]
            self.episode_rewards.append(episode_reward)

            if len(self.episode_rewards) >= self.check_freq:

                mean_reward = np.mean(self.episode_rewards)
                std_reward = np.std(self.episode_rewards)

                self.reward_avg.append(mean_reward)
                self.reward_std.append(std_reward)

                ## SAVE MODEL IF HIGHEST MEAN REWARD ACHIEVED
                if mean_reward > self.highest_reward and self.num_timesteps >= 20e3:

                    ## SAVE HIGHEST REWARD
                    self.highest_reward = mean_reward
                    print(f"New high score ({self.highest_reward:.3f}), model saved.")

                    ## SAVE NEWEST MODEL AND REPLAY BUFFER
                    newest_model = os.path.join(self.model_dir, f"{self.num_timesteps}_step_model")
                    newest_replay_buff = os.path.join(self.model_dir, f"{self.num_timesteps}_step_replay_buff")

                    self.model.save(newest_model)
                    self.model.save_replay_buffer(newest_replay_buff)

                ## RESET EPISODIC REWARD LIST
                self.episode_rewards = []

            ## TB LOGGING VALUES
            self.logger.record('custom/K_ep',self.training_env.envs[0].env.K_ep)
            self.logger.record('custom/Reward',episode_reward.item())
            self.logger.record('custom/Reward_avg',self.reward_avg[-1])


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
            self.TB_log_path = os.path.join(log_dir,f"{self.log_name}_0")
            self.model_dir = os.path.join(self.TB_log_path,"models")

        ## GENERATE LOG/MODEL DIRECTORY
        if not os.path.exists(self.TB_log_path):
            os.makedirs(self.TB_log_path,exist_ok=True)

        if not os.path.exists(self.model_dir):
            os.makedirs(self.model_dir,exist_ok=True)

    def create_model(self,gamma=0.999,learning_rate=0.002,net_arch=[8,8]):
        """Create Soft Actor-Critic agent used in training

        Args:
            gamma (float, optional): Discount factor. Defaults to 0.999.
            learning_rate (float, optional): Learning Rate. Defaults to 0.002.
            net_arch (list, optional): Network layer sizes and architechure. Defaults to [12,12].
        """        

        self.model = SAC(
            "MlpPolicy",
            env=self.env,
            gamma=gamma,
            learning_rate=learning_rate,
            policy_kwargs=dict(activation_fn=th.nn.ReLU,net_arch=dict(pi=net_arch, qf=[256,256])),
            verbose=1,
            device='cpu',
            tensorboard_log=self.log_dir
        ) 

        if self.env.Env_Name != "CF_Env_2D":
            self.save_config_file()

        return self.model
    
    def load_model(self,model_dir,model_name,t_step):
        """Loads current model and replay buffer from the selected time step

        Args:
            t_step (int): Timestep of the selected model
        """        

        ## MODEL PATHS
        model_path = os.path.join(model_dir,model_name,"models",f"{t_step}_step_model")
        replay_buff_path = os.path.join(model_dir,model_name,"models",f"{t_step}_step_replay_buff")

        ## LOAD MODEL AND REPLAY BUFFER
        self.model = SAC.load(
            model_path,
            env=self.env,
            device='cpu',
            tensorboard_log=self.log_dir
        )
        self.model.load_replay_buffer(replay_buff_path)

    def load_params_from_model(self,model_dir,model_name,t_step):

        ## MODEL PATHS
        model_path = os.path.join(model_dir,model_name,"models",f"{t_step}_step_model")

        ## LOAD PREV MODEL AND REPLAY BUFFER
        prev_model = SAC.load(
            model_path,
            env=None,
            device='cpu',
        )

        ## TRANSFER PARAMETERS TO CURRENT MODEL
        self.model.policy.actor = prev_model.actor
        self.model.policy.critic = prev_model.critic
   
    def train_model(self,check_freq=10,save_freq=5e3,reset_timesteps=False,total_timesteps=2e6):
        """Script to train model via DeepRL method

        Args:
            check_freq (int): Number of episodes to average rewards over. Defaults to 10.
            save_freq (int): Number of timesteps to save model. Defaults to 5e3.
            reset_timesteps (bool, optional): Reset starting timestep to zero. Defaults to True. 
                                              Set to False to resume training from previous model.
            total_timesteps (int, optional): Timesteps to cutoff model training. Defaults to 2e6.
        """        
        
        reward_callback = RewardCallback(check_freq=check_freq,save_freq=save_freq,model_dir=self.model_dir)

        self.model.learn(
            total_timesteps=int(total_timesteps),
            tb_log_name=self.log_name,
            callback=reward_callback,
            reset_num_timesteps=reset_timesteps,
        )

    def save_config_file(self):

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

            SAR_SETTINGS = dict(
                SAR_Type = self.env.SAR_Type,
                SAR_Config = self.env.SAR_Config,
            ),

            ENV_SETTINGS = dict(
                Environment = self.env.Env_Name,
                Vel_Limts = self.env.Vel_range,
                Phi_Limits = self.env.Phi_rel_range,
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
                Activation_Function = "",
            )


        )

        with open(config_path, 'w') as outfile:
            yaml.dump(data,outfile,default_flow_style=False,sort_keys=False)

    def test_policy(self,Vel=None,Phi=None,episodes=10):
        """Test the currently loaded policy for a given set of velocity and launch angle conditions

        Args:
            Vel (float, optional): Flight velocity [m/s]. 
            Phi (float, optional): Flight angle [deg]. 
            episodes (int, optional): Number of landing tests. Defaults to 1.
        """        

        for ep in range(episodes):
            obs,_ = self.env.reset(Vel=Vel,Phi=Phi)
            done = False
            while not done:
                action,_ = self.model.predict(obs)
                obs,reward,done,_,_ = self.env.step(action)

            print(f"Episode {ep}: Reward {reward:.3f}")

    def collect_landing_performance(self,fileName=None,Vel_inc=0.25,Phi_inc=5,n_episodes=5):
        """Test trained model over varied velocity and flight angle combinations.

        Args:
            fileName (str, optional): fileName to save logged CSV as. Defaults to None.
            Vel_inc (float, optional): Flight velocity increment [m/s]. Defaults to 0.25.
            Phi_inc (int, optional): Flight angle increment [deg]. Defaults to 5.
            n_episodes (int, optional): Number of episodes to test each velocity over. Defaults to 5.
        """        

        if fileName == None:
            fileName = "PolicyPerformance_Data.csv"
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
                    obs,_ = self.env.reset(Vel=Vel,Phi=Phi)
                    done = False
                    while not done:
                        action,_ = self.model.predict(obs)
                        obs,reward,done,_,_ = self.env.step(action)


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

                            np.round(self.env.obs_trg[0],3),
                            np.round(self.env.obs_trg[1],3),
                            np.round(self.env.obs_trg[2],3),

                            np.round(self.env.eul_impact[1],3),

                            np.round(self.env.action_trg[0],3),
                            np.round(self.env.action_trg[1],3),

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

    def plot_landing_performance(self,fileName=None,saveFig=False):

        if fileName == None:
            fileName = "PolicyPerformance_Data.csv"
        filePath = os.path.join(self.TB_log_path,fileName)

        af = pd.read_csv(filePath,sep=',',comment="#")

        af2 = af.groupby(['Vel_d','Phi_d']).mean().round(3).reset_index()



        ## COLLECT DATA
        R = af2.iloc[:]['Vel_d']
        Theta = af2.iloc[:]['Phi_d']
        C = af2.iloc[:]['4_Leg_NBC']

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
        ax.set_rmax(4.0)
        


        # ## AXIS LABELS    
        # ax.text(np.radians(7.5),2,'Flight Velocity (m/s)',
        #     rotation=18,ha='center',va='center')

        # ax.text(np.radians(60),4.5,'Flight Angle (deg)',
        #     rotation=0,ha='left',va='center')

        if saveFig==True:
            plt.savefig(f'{self.TB_log_path}/Landing_Rate_Fig.pdf',dpi=300)

        plt.show()

    def policy_output(self,obs):
        """Returns direct output of policy network in form of action_mean and action_log_std.
        Policy gaussians are made from action_mean and action_std. 

        Args:
            obs (list): Observation vector returned by environment

        Returns:
            action_mean: Action distribution means
            action_log_std: Action distribution log_stds (exponentiate to get normal std values)
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

        ## CONVERT LOG_STD TO STD
        action_log_std = log_std
        action_log_std = action_log_std.detach().numpy()[0]

        ## GRAB ACTION DISTRIBUTION MEAN
        action_mean = mean_actions
        action_mean = action_mean.detach().numpy()[0]

        return action_mean,action_log_std
    
    def save_NN_Params(self):
        """Save NN parameters to C header file for upload to crazyflie

        Args:
            SavePath (str): Path to save header file
        """        

        FileName = f"NN_Layers_{self.env.modelInitials}_DeepRL.h"
        f = open(os.path.join(self.TB_log_path,FileName),'a')
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

if __name__ == '__main__':


    ## IMPORT ENVIRONMENTS
    from Envs.SAR_Sim_DeepRL import SAR_Sim_DeepRL
    from Envs.CF_Env_2D import CF_Env_2D


    # START TRAINING NEW DEEP RL MODEL 
    env = SAR_Sim_DeepRL(GZ_Timeout=True,Vel_range=[3.0,4.0],Phi_rel_range=[0,180])
    log_name = f"{env.modelInitials}--Deg_{env.Plane_Angle}--SAC_{current_time}" 
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/{env.Env_Name}"


    # # env = CF_Env_2D(Vel_range=[2.0,4.0],Phi_rel_range=[0,90])
    # # # env.RENDER = True
    # # log_name = "Testing_Log"
    # # log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/{env.Env_Name}"



    PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    PolicyTrainer.create_model()
    PolicyTrainer.train_model(save_freq=5000)


    # ================================================================= ##
    
    # # # RESUME TRAINING DEEP RL MODEL
    # log_name = "A30_L75_K08--Deg_180.0--SAC_07_06-12:50_0"
    # t_step_load = 10000
    # env.RENDER = True

    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.load_model(log_dir,log_name,t_step_load)
    # # # PolicyTrainer.train_model(save_freq=5000,total_timesteps=60000)
    # PolicyTrainer.test_policy(episodes=30)

    # # # PolicyTrainer.collect_landing_performance()
    # # # PolicyTrainer.plot_landing_performance(saveFig=True)



    # ================================================================= ##

    # COLLECT LANDING PERFORMANCE DATA
    # env = SAR_Sim_DeepRL(GZ_Timeout=False,My_range=[0,8],Vel_range=[1.5,4.0],Phi_rel_range=[0,90])
    # log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_Sim_DeepRL_Env"
    # log_name = "A30_L75_K08--Deg_180.0--SAC_07_01-18:54_0"
    # t_step_load = 55488

    # PolicyTrainer = Policy_Trainer_DeepRL(env,log_dir,log_name)
    # PolicyTrainer.load_model(log_dir,log_name,t_step_load)
    # PolicyTrainer.test_policy(Vel=2.5,Phi=60,episodes=10)


    