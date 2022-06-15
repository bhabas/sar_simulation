from stable_baselines3 import PPO
from Env_Example import CustomEnv


## SELECT MODEL FROM DIRECTORY
models_dir = "crazyflie_projects/DeepRL/models/PPO-14_34"
model_path = f"{models_dir}/190_000.zip"

## INITIATE ENVIRONMENT AND TRAINED MODEL
env = CustomEnv()
env.reset()
model = PPO.load(model_path,env=env)

## RENDER TRAINED MODEL FOR N EPISODES
episodes = 10
for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()