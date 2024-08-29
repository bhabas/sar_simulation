import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Load the data from the CSV file
data = pd.read_csv('/home/bhabas/Downloads/Reward_Plot.csv')
episodes = data['Episode'].to_numpy()
rewards = (data['Reward'].to_numpy()-0.2)*2.2

# Smooth the rewards using a moving average
window_size = 20  # Adjust the window size for more or less smoothing
smoothed_rewards = np.convolve(rewards, np.ones(window_size)/window_size, mode='valid')

# Adjust the episodes to match the smoothed rewards
smoothed_episodes = episodes[window_size-1:]

# Create the figure and axis
fig = plt.figure(figsize=(8, 4))
ax = fig.add_subplot(111)
line = ax.plot(smoothed_episodes, smoothed_rewards, alpha=0.4, color="tab:orange")[0]  # Plot the smoothed rewards
line_ani, = ax.plot([], [], color="tab:orange")  # Initialize the line
dot_ani, = ax.plot([], [], color="tab:orange", marker="o")  # Initialize the dot

# Set labels and title
ax.set_xlabel('Episode')
ax.set_ylabel('Reward')
ax.set_title('Reward vs Episode')

# Set y-axis range and ticks
ax.set_ylim(0, 1.1)
ax.set_yticks(np.arange(0, 1.25, 0.25))  # Labels every 0.25
ax.set_xlim(0,4000)

# Initialize the plot limits
ax.set_ylim(0, 1.1)
ax.grid()

# Function to initialize the animation
def init():
    line_ani.set_data([], [])
    dot_ani.set_data([], [])
    return line_ani, dot_ani

# Function to update the line and dot in the animation
def update(frame):
    # Update the line with the current frame
    line_ani.set_data(smoothed_episodes[:frame], smoothed_rewards[:frame])
    # Update the dot to follow the line
    dot_ani.set_data(smoothed_episodes[frame], smoothed_rewards[frame])
    return line_ani, dot_ani

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=len(smoothed_episodes), init_func=init, interval=20, blit=True)

# Save the animation as an mp4 file
ani.save('/home/bhabas/Downloads/reward_vs_episode_animation.mp4', writer='ffmpeg', fps=30,dpi=300)

# plt.show()
