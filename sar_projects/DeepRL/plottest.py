import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot()



# Assuming these are your initial lists and they will be updated
Vx_list = np.random.uniform(0.0, 5.0, 1000)
Vz_list = np.random.uniform(0.0, 5.0, 1000)
rew_list = np.random.uniform(0.0, 1.0, 1000)

# Initial last processed index, -1 indicates no data has been processed yet
last_processed_idx = 0

# Discretize the space
Vx_bins = np.arange(0,5,step=0.25)
Vz_bins = np.arange(-5.0,5,step=0.25)
reward_grid = np.full((len(Vx_bins), len(Vz_bins)), np.nan)



def update_reward_grid(Vx_list, Vz_list, rew_list, last_processed_idx, reward_grid):
    # Process only new data points added since the last update
    for Vx, Vz, reward in zip(Vx_list[last_processed_idx:], Vz_list[last_processed_idx:], rew_list[last_processed_idx:]):
        Vx_idx = np.digitize(Vx, Vx_bins) - 1
        Vz_idx = np.digitize(Vz, Vz_bins) - 1
        last_processed_idx += 1
        if 0 <= Vx_idx < len(Vx_bins)-1 and 0 <= Vz_idx < len(Vz_bins)-1:
            reward_grid[Vx_idx, Vz_idx] = reward

    return last_processed_idx, reward_grid

last_processed_idx, reward_grid = update_reward_grid(Vx_list, Vz_list, rew_list, last_processed_idx, reward_grid)


Vx_list = np.hstack((Vx_list, np.random.uniform(0.0, 5.0, 10)))
Vz_list = np.hstack((Vz_list, np.random.uniform(-5.0, 0.0, 10)))
rew_list = np.hstack((rew_list, np.random.uniform(0.0, 1.0, 10)))
last_processed_idx, reward_grid = update_reward_grid(Vx_list, Vz_list, rew_list, last_processed_idx, reward_grid)


# Plot the data
cmap = plt.cm.jet
ax.imshow(reward_grid.T, interpolation='nearest', cmap=cmap, extent=[0.0,5.0,-5.0,5.0], aspect='equal', origin='lower',zorder=0)

# Define grid lines to appear behind the heatmap
ax.set_xticks(Vx_bins, minor=True)
ax.set_yticks(Vz_bins, minor=True)


ax.set_xlim(0.0, 5.0)
ax.set_ylim(-5.0, 5.0)


ax.set_xlabel('Vx (m/s)')
ax.set_ylabel('Vz (m/s)')
ax.set_title('Reward Bins')


plt.show(block=True)
