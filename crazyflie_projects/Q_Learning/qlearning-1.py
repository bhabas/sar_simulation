from dis import dis
import gym
import numpy as np
from sympy import Le, false

env = gym.make("MountainCar-v0")
env.reset()

LEARNING_RATE =  0.1
DISCOUNT_RATE = 0.95 # How important we count future reward vs current rewards
EPISODES = 25_000

epsilon = 0.5 # Exploration factor
START_EPSILON_DECAYING = 1
END_EPSILONG_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILONG_DECAYING-START_EPSILON_DECAYING)


DISCRETE_OS_SIZE = [20]*len(env.observation_space.high)
discrete_os_win_size = (env.observation_space.high - env.observation_space.low) / DISCRETE_OS_SIZE

q_table = np.random.uniform(low=-2,high=0,size=(DISCRETE_OS_SIZE + [env.action_space.n]))

def get_discrete_state(state):
    discrete_state = (state - env.observation_space.low) / discrete_os_win_size
    return tuple(discrete_state.astype(np.int64))



for episode in range(EPISODES):

    if episode%500 == 0:
        render = True
        print(episode)
    else:
        render = False

    discrete_state = get_discrete_state(env.reset())
    done = False
    while not done:

        if np.random.random() > epsilon:
            action = np.argmax(q_table[discrete_state])
        else:
            action = np.random.randint(0,env.action_space.n)

        new_state,reward,done,_ = env.step(action)

        new_discrete_state = get_discrete_state(new_state)

        if render:
            env.render()

        if not done:
            max_future_q = np.max(q_table[new_discrete_state])
            current_q = q_table[discrete_state + (action,)]

            new_q = (1 - LEARNING_RATE)*current_q + LEARNING_RATE*(reward + DISCOUNT_RATE*max_future_q)

            ## UPDATE Q-TABLE VALUE
            q_table[discrete_state + (action,)] = new_q

        elif new_state[0] >= env.goal_position:
            q_table[discrete_state + (action,)] = 0
            print(f"We made it on episode: {episode}")

        discrete_state = new_discrete_state

    if END_EPSILONG_DECAYING >= episode >= START_EPSILON_DECAYING:
        epsilon -= epsilon_decay_value

env.close()