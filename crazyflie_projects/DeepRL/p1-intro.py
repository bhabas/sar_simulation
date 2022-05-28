import gym
from stable_baselines3 import A2C,PPO

env = gym.make("LunarLanderConti-v2")

env.reset()

model = PPO("MlpPolicy",env,verbose=1) # Multilayer Perceptron (Mlp)

## INTERACT WITH ENV AND TRAIN MODEL
model.learn(total_timesteps=500_000)

## PLAY TRAINED MODEL

while True:
    obs = env.reset()
    done = False
    while not done:
        env.render()
        action,_states = model.predict(obs)
        obs,rewards,done,info = env.step(action)

# env.close()