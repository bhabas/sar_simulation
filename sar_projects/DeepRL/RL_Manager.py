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
        
    def load_model(self,model_dir,t_step: int):

        model_path = os.path.join(model_dir, f"model_{int(t_step)}_steps")
        replay_buffer_path = os.path.join(model_dir, f"replay_buffer_{int(t_step)}_steps")

        ## LOAD MODEL AND REPLAY BUFFER
        self.model = SAC.load(
            model_path,
            env=self.vec_env,
            device='cpu',
            tensorboard_log=self.log_dir,
        )
        self.model.load_replay_buffer(replay_buffer_path)

    def train_model(self,check_freq=10,save_freq=1e3,reset_timesteps=True,total_timesteps=2e6):

        reward_callback = RewardCallback(check_freq=check_freq,save_freq=save_freq,model_dir=self.model_dir)

        self.model.learn(
            total_timesteps=int(total_timesteps),
            callback=reward_callback,
            tb_log_name="TB_Log",
            reset_num_timesteps=reset_timesteps,
        )

    def test_policy(self,V_mag=None,V_angle=None,Plane_Angle=None):

        # self.vec_env.Render_Flag = True
        self.env.setTestingConditions(V_mag=V_mag,V_angle=V_angle,Plane_Angle=Plane_Angle)
        obs = self.vec_env.reset()
        terminated = False
 
        while not terminated:
            action,_ = self.model.predict(obs)
            obs,reward,terminated,_ = self.vec_env.step(action)

        return obs[0],reward[0]
    
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

        ## TIME ESTIMATION FILTER INITIALIZATION
        num_trials = len(V_mag_arr)*len(V_angle_arr)*len(Plane_Angle_arr)*n_episodes
        idx = 0
        t_delta = 0
        t_delta_prev = 0

        if not os.path.exists(filePath):
            with open(filePath,'w') as file:
                writer = csv.writer(file,delimiter=',')
                writer.writerow([
                    "V_mag_d", "V_angle_d", "Plane_Angle", "Trial_num",

                    "--",

                    "Pad_Connections",
                    "BodyContact","ForelegContact","HindlegContact",

                    "--",
                    "Policy_Trg_Action_trg",
                    "Policy_Rot_Action_trg",
                    "Vel_mag_B_O_trg","Vel_angle_B_O_trg",
                    "Vel_mag_B_P_trg","Vel_angle_B_P_trg",

                    "Tau_CR_trg",
                    "Tau_trg"
                    "Theta_x_trg",
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
                ])

        for Plane_Angle in Plane_Angle_arr:
            for V_angle in V_angle_arr:
                for V_mag in V_mag_arr:
                    for trial in range(n_episodes):

                        start_time = time.time()

                        ## TEST POLICY FOR GIVEN FLIGHT CONDITIONS
                        self.test_policy(V_mag,V_angle,Plane_Angle)

                        ## APPEND RECORDED VALUES TO CSV FILE
                        with open(filePath,'a') as file:
                            writer = csv.writer(file,delimiter=',',quoting=csv.QUOTE_NONE,escapechar='\\')
                            writer.writerow([
                                V_mag,V_angle,Plane_Angle,trial,
                                "--",
                                self.env.Pad_Connections,
                                self.env.BodyContact_Flag,self.env.ForelegContact_Flag,self.env.HindlegContact_Flag,
                                "--",
                                self.env.Policy_Trg_Action_trg,
                                self.env.Policy_Rot_Action_trg,
                                self.env.Vel_mag_B_O_trg,self.env.Vel_angle_B_O_trg,
                                self.env.Vel_mag_B_P_trg,self.env.Vel_angle_B_P_trg,
                                self.env.Tau_CR_trg,
                                self.env.Tau_trg,
                                self.env.Theta_x_trg,
                                self.env.D_perp_trg,
                                "--",
                                self.env.Eul_B_O_impact_Ext[2],
                                self.env.Eul_P_B_impact_Ext[2],
                                self.env.Omega_B_P_impact_Ext[2],
                                self.env.V_B_P_impact_Ext[0],self.env.V_B_P_impact_Ext[2],
                                self.env.Impact_Magnitude,
                                self.env.Force_impact_x,self.env.Force_impact_y,self.env.Force_impact_z,
                                "--",
                                self.env.reward,np.round(self.env.reward_vals,3),
                            ])

                            ## CALCULATE AVERAGE TIME PER EPISODE
                            t_delta = time.time() - start_time
                            t_delta_avg = EMA(t_delta,t_delta_prev,alpha=0.01)
                            t_delta_prev = t_delta_avg
                            idx += 1

                            TTC = round(t_delta_avg*(num_trials-idx)) # Time to completion
                            print(f"Flight Conditions: ({V_mag:.02f} m/s,{V_angle:.02f} deg) \t Index: {idx}/{num_trials} \t Percentage: {100*idx/num_trials:.2f}% \t Time to Completion: {str(timedelta(seconds=TTC))}")

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

    def policy_output(self,obs):
 
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




class RewardCallback(BaseCallback):
    def __init__(self, check_freq: int, save_freq: int, model_dir: str, verbose=1):
        super(RewardCallback, self).__init__(verbose)
        self.check_freq = check_freq
        self.save_freq = save_freq
        self.model_dir = model_dir

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        pass

    def _on_step(self) -> bool:

        ## SAVE MODEL EVERY N TIMESTEPS
        if self.n_calls % self.save_freq == 0:

            ## SAVE NEWEST MODEL AND REPLAY BUFFER
            model_path = os.path.join(self.model_dir, f"model_{self.num_timesteps}_steps")
            replay_buffer_path = os.path.join(self.model_dir, f"replay_buffer_{self.num_timesteps}_steps")

            self.model.save(model_path)
            self.model.save_replay_buffer(replay_buffer_path)
        
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
    current_time = current_datetime.strftime("%H:%M:%S")
    log_name = f"DeepRL_Policy_{current_time}"
    log_dir = f"{BASE_PATH}/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL" 

    # ================================================================= ##

    # Define the environment parameters
    env_kwargs = {
        "Ang_Acc_range": [-100, 0],
        "V_mag_range": [1.0, 2.5],
        "V_angle_range": [60, 60],
        "Plane_Angle_range": [0, 0],
        "Render": True
    }


    log_name = "DeepRL_Policy_09:02:31"
    model_dir = "/home/bhabas/catkin_ws/src/sar_simulation/sar_projects/DeepRL/TB_Logs/SAR_2D_DeepRL/DeepRL_Policy_09:02:31/Models"
    
    RL_Manager = RL_Training_Manager(SAR_2D_Env,log_dir,log_name,env_kwargs=env_kwargs)
    # RL_Manager.create_model(net_arch=[14,14,14])
    RL_Manager.load_model(model_dir,t_step=20e3)
    # RL_Manager.save_NN_to_C_header()
    obs = [0.214436,1.837226,0.68069,0]
    
    print(RL_Manager.policy_output(obs))




    # RL_Manager.sweep_policy(Plane_Angle_range=[0,0,1],V_angle_range=[60,60,1],V_mag_range=[1.0,3.0,5],n=3)
    # RL_Manager.collect_landing_performance(
    #     fileName="PolicyPerformance_Data.csv",
    #     V_mag_step=0.5,
    #     V_angle_step=5,
    #     Plane_Angle_step=45,
    #     n_episodes=3
    #     )
    # RL_Manager.train_model()
