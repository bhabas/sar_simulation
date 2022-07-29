from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
import torch as th
import numpy as np


## ADD CRAZYFLIE_SIMULATION DIRECTORY TO PYTHONPATH SO ABSOLUTE IMPORTS CAN BE USED
import sys,rospkg,os
BASE_PATH = os.path.dirname(rospkg.RosPack().get_path('crazyflie_logging'))
sys.path.insert(1,'/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_env')
sys.path.insert(1,BASE_PATH)

from crazyflie_env.CrazyflieEnv_DeepRL import CrazyflieEnv_DeepRL
from CF_Env_2D import CF_Env_2D
from CF_Env_2D_dTau import CF_Env_2D_dTau
from CF_Env_2D_Simple import CF_Env_2D_Simple

## COLLECT CURRENT TIME
now = datetime.now()
current_time = now.strftime("%H-%M")


def save_NN_Params(SavePath,FileName,model):
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
    for module in model.actor.latent_pi.modules():
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
    for module in model.actor.mu.modules():
        W_mu = module.weight.detach().numpy()
        b_mu = module.bias.detach().numpy().reshape(-1,1)

    for module in model.actor.log_std.modules():
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


def custom_predict(obs):

    # CAP the standard deviation of the actor
    LOG_STD_MAX = 2
    LOG_STD_MIN = -20
    actor = model.policy.actor
    obs = th.tensor([obs])
    sub_action = model.policy.actor.forward(obs)

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
    low, high = env.action_space.low, env.action_space.high
    return low + (0.5 * (scaled_action + 1.0) * (high - low))


if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CrazyflieEnv_DeepRL()


    ## CREATE MODEL AND LOG DIRECTORY
    log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
    log_name = f"SAC-07_28-19:32_0"
    NN_path = os.path.join(log_dir,log_name)
    NN_FileName = "NN_Layers_NL_DeepRL.h"
    model_path = os.path.join(log_dir,log_name,f"models/{53}000_steps.zip")
    model = SAC.load(model_path,env=env,device='cpu')

    # policy_kwargs = dict(activation_fn=th.nn.ReLU,
    #                  net_arch=[4, 4])
    # model = SAC(
    #     "MlpPolicy",
    #     env=env,
    #     gamma=0.999,
    #     learning_rate=0.001,
    #     policy_kwargs=policy_kwargs,
    #     verbose=1,
    #     device='cpu',
    #     tensorboard_log=log_dir
    # ) 


    save_NN_Params(NN_path,NN_FileName,model)
    # obs = np.array([ 0.1655, -2.3880,  0.3551],dtype=np.float32)

    # while True:
    #     action = custom_predict(obs)[0]
    #     action[0] = np.arctanh(action[0])
    #     print(action)

    ## RENDER TRAINED MODEL FOR N EPISODES-
    # episodes = 50
    # env.RENDER = True
    # for ep in range(episodes):
    #     obs = env.reset()
    #     done = False
    #     while not done:
    #         env.render()
    #         action = custom_predict(obs)[0]
    #         # action,_ = model.predict(obs)
    #         obs,reward,done,info = env.step(action)

    # env.close()