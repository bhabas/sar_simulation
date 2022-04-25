import gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

env = gym.make("MountainCar-v0")
env.reset()

## LEARNING FACTORS
LR = 0.1
DISCOUNT_RATE = 0.99
EPISODES = 10_000

## EXPLORATION FACTORS
epsilon = 0.5
START_EPSILON_DECAYING = 1
END_EPSILONG_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILONG_DECAYING-START_EPSILON_DECAYING)


## DISCRETIZE CONTINUOUS STATES
num_bins = 20
bin_size = (env.observation_space.high - env.observation_space.low)/num_bins


def get_state_index(state): # Return index of current state 
    discrete_state = (state-env.observation_space.low)//bin_size
    return tuple(discrete_state.astype(np.int64))

## INITIALIZE Q-TABLE
Q_table = np.zeros((num_bins,num_bins,env.action_space.n))
Q_table[:,:,1] = 2
# pass



## INITIALIZE REWARD LOGGING
ep_rewards = []
aggr_ep_rewards = {
    'ep':[], 
    'avg':[], 
    'min':[],
    'max':[]}

for episode in range(EPISODES):

    ## VISUAL RENDERING
    if episode%500 == 0:
        render = True
    else:
        render = False

    ## INITIALIZE EPISODE
    episode_reward = 0
    discrete_state = get_state_index(env.reset())
    done = False

    while not done:

        ## PERFORM OPTIMAL ACTION/EXPLORATION ACTION
        if np.random.random() >= epsilon:
            action = np.argmax(Q_table[discrete_state])
        else:
            action = np.random.randint(0,env.action_space.n)

        ## UPDATE ENVIRONMENT
        new_state,reward,done,_ = env.step(action)
        new_discrete_state = get_state_index(new_state)
        episode_reward += reward

        # if render == True:
        #     env.render()

        if not done:

            ## UPDATE Q-TABLE
            Q_current = Q_table[discrete_state][action]
            Q_max_future = np.max(Q_table[new_discrete_state])
            Q_current = Q_current + LR*(reward + DISCOUNT_RATE*Q_max_future - Q_current)
            Q_table[discrete_state][action] = Q_current

        elif new_state[0] >= env.goal_position:
            Q_table[discrete_state][action] = 0

        ## UPDATE CURRENT STATE
        discrete_state = new_discrete_state

        ## REDUCE EPSILON AS EPISODE COUNT INCREASES
        if START_EPSILON_DECAYING <= episode <= END_EPSILONG_DECAYING:
            epsilon -= epsilon_decay_value
            
    ## LOG EPISODE TOTAL REWARD
    ep_rewards.append(episode_reward)

    if episode%100 == 0:

        average_reward = np.average(ep_rewards[-100:])
        aggr_ep_rewards['ep'].append(episode)
        aggr_ep_rewards['avg'].append(average_reward)
        aggr_ep_rewards['min'].append(min(ep_rewards[-100:]))
        aggr_ep_rewards['max'].append(max(ep_rewards[-100:]))

        print(f"Episode: {episode} avg: {average_reward} min: {min(ep_rewards[-100:])} max: {max(ep_rewards[-100:])}")

env.close()


fig = plt.figure()
ax = fig.add_subplot(111)

ax.imshow(Q_table[:,:].argmax(axis=2),
    cmap=cm.coolwarm,
    vmin=0,vmax=2,
    extent=(-1.2,0.6,-0.07,0.07),
    aspect='auto')
ax.set_title("Q-Table State-Action pairs")
ax.set_xlabel("Position [m]")
ax.set_ylabel("Velocity [m/s]")


plt.show()

fig = plt.figure()
ax = fig.add_subplot(111)

ax.plot(aggr_ep_rewards['ep'],aggr_ep_rewards['avg'], label="avg")
ax.plot(aggr_ep_rewards['ep'],aggr_ep_rewards['min'], label="min")
ax.plot(aggr_ep_rewards['ep'],aggr_ep_rewards['max'], label="max")
plt.legend()
plt.show()
