import gym
import numpy as np
import matplotlib.pyplot as plt

env = gym.make("MountainCar-v0")
env.reset()


## LEARNING FACTORS
LEARNING_RATE =  0.1
DISCOUNT_RATE = 0.95 # How important we count future reward vs current rewards
EPISODES = 5000
epsilon = 0.5 # Exploration factor


## EXPLORATION DECAY FACTORS
START_EPSILON_DECAYING = 1
END_EPSILONG_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILONG_DECAYING-START_EPSILON_DECAYING)

## VALUE DISCRETIZATION
DISCRETE_OS_SIZE = [20]*len(env.observation_space.high)
discrete_os_win_size = (env.observation_space.high - env.observation_space.low) / DISCRETE_OS_SIZE

## GENERATE INITIAL Q-TABLE
q_table = np.random.uniform(low=-2,high=0,size=(DISCRETE_OS_SIZE + [env.action_space.n]))
ep_rewards = []
aggr_ep_rewards = {'ep':[], 'avg':[], 'min':[], 'max':[]}

def get_discrete_state(state):
    discrete_state = (state - env.observation_space.low) / discrete_os_win_size
    return tuple(discrete_state.astype(np.int64))



for episode in range(EPISODES):

    ## VISUAL RENDERING
    if episode%500 == 0:
        render = True
    else:
        render = False

    ## EPISODE INITIALIZATION
    episode_reward = 0
    discrete_state = get_discrete_state(env.reset())
    done = False

    while not done:
        
        ## EPSILON-GREEDY IMPLEMENTATION
        if np.random.random() > epsilon:
            action = np.argmax(q_table[discrete_state])
        else:
            action = np.random.randint(0,env.action_space.n)

        ## UPDATE ENVIRONMENT
        new_state,reward,done,_ = env.step(action)
        new_discrete_state = get_discrete_state(new_state)
        episode_reward += reward


        if render:
            env.render()

        if not done:

            ## UPDATE Q-TABLE WITH NEW Q-VALUE FOR CURRENT STATE
            current_q = q_table[discrete_state + (action,)]
            max_future_q = np.max(q_table[new_discrete_state])
            new_q = (1 - LEARNING_RATE)*current_q + LEARNING_RATE*(reward + DISCOUNT_RATE*max_future_q)
            q_table[discrete_state + (action,)] = new_q

        elif new_state[0] >= env.goal_position:
            q_table[discrete_state + (action,)] = 0

        ## UPDATE CURRENT STATE
        discrete_state = new_discrete_state

    ## REDUCE EPSILON VALUE AS EPISODE NUMBER INCREASES
    if START_EPSILON_DECAYING <= episode <= END_EPSILONG_DECAYING:
        epsilon -= epsilon_decay_value

    ep_rewards.append(episode_reward)

    if episode%500 == 0:

        average_reward = np.average(ep_rewards[-500:])
        aggr_ep_rewards['ep'].append(episode)
        aggr_ep_rewards['avg'].append(average_reward)
        aggr_ep_rewards['min'].append(min(ep_rewards[-500:]))
        aggr_ep_rewards['max'].append(max(ep_rewards[-500:]))

        print(f"Episode: {episode} avg: {average_reward} min: {min(ep_rewards[-500:])} max: {max(ep_rewards[-500:])}")


env.close()

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(aggr_ep_rewards['ep'],aggr_ep_rewards['avg'], label="avg")
ax.plot(aggr_ep_rewards['ep'],aggr_ep_rewards['min'], label="min")
ax.plot(aggr_ep_rewards['ep'],aggr_ep_rewards['max'], label="max")
plt.legend()
plt.show()
