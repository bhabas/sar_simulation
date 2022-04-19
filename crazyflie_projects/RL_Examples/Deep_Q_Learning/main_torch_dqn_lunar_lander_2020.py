import gym
from simple_dqn_torch_2020 import Agent
from utils import plot_learning_curve
import numpy as np


if __name__ == "__main__":
    env = gym.make('LunarLander-v2')
    agent = Agent(gamma=0.99, epsilon=1.0, batch_size=64, n_actions=4,
                eps_end=0.01, input_dims=[8], lr=0.001)
        
    scores = []
    eps_history = []
    n_games =  500

    for i in range(n_games):
        score = 0
        done = False
        observation = env.reset()
        while not done:
            action = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            score += reward
            agent.store_transition(observation, action, reward, observation_, done)
            agent.learn()
            observation = observation_
        
        scores.append(score)
        eps_history.append(agent.epsilon)

        avg_score = np.mean(scores[-100:])
        print(f"episode: {i} score: {score:.2f} average score: {avg_score:.2f} epsilon {agent.epsilon:.2f}")

    x = [i+1 for i in range(n_games)]
    filename = 'lunear_lander_2020.png'
    plot_learning_curve(x, scores, eps_history, filename)