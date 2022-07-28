from datetime import datetime
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import *
import torch as th
import numpy as np
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
    
    NN_size = np.array([4]).reshape(-1,1)

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

    ## SAVE NN LAYER VALUES
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

    for module in model.actor.mu.modules():
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


    f.write("};")
    f.close()


def custom_predict(obs):

    # CAP the standard deviation of the actor
    LOG_STD_MAX = 2
    LOG_STD_MIN = -20
    actor = model.policy.actor
    obs = th.tensor([obs])
    model.policy.actor.forward(obs)

    ## PASS OBS THROUGH NN
    latent_pi = actor.latent_pi(obs)
    mean_actions = actor.mu(latent_pi)
    log_std = actor.log_std(latent_pi)
    log_std = th.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)

    ## CONVERT MU/LOG_STD TO NORMAL DISTRIBUTION AND SAMPLE
    action_std = th.ones_like(mean_actions) * log_std.exp()
    action_mean = mean_actions
    # samples = th.normal(action_mean,action_std)
    # scaled_action = th.tanh(samples).detach().numpy()

    ## CENTRAL LIMIT THEOREM SAMPLE
    normal_sample = np.sum(np.random.uniform(size=(12,2)),axis=0)-6
    samples = normal_sample*action_std.detach().numpy()+action_mean.detach().numpy()
    scaled_action = np.tanh(samples)

    ## SQUISH SAMPLES TO [-1,1] RANGE AND RESCALE
    low, high = env.action_space.low, env.action_space.high
    return low + (0.5 * (scaled_action + 1.0) * (high - low))


if __name__ == '__main__':

    ## INITIATE ENVIRONMENT
    env = CF_Env_2D()


    ## CREATE MODEL AND LOG DIRECTORY
    log_dir = f"/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/DeepRL/logs/{env.env_name}"
    log_name = f"SAC-11-09_0"
    NN_path = os.path.join(log_dir,log_name)
    NN_FileName = "NN_Layers_NL_DeepRL.h"
    model_path = os.path.join(log_dir,log_name,f"models/{999}000_steps.zip")
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

    # ## RENDER TRAINED MODEL FOR N EPISODES-
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