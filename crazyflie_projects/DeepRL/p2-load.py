import gym
from stable_baselines3 import PPO
from Env_Example import CustomEnv

# env = gym.make("CartPole-v1")
env = CustomEnv()
env.reset()

models_dir = "crazyflie_projects/DeepRL/models/PPO4"
model_path = f"{models_dir}/50000.zip"

model = PPO.load(model_path,env=env)

episodes = 10

for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_ = model.predict(obs)
        obs,reward,done,info = env.step(action)

env.close()