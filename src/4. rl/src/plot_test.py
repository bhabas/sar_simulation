import matplotlib.pyplot as plt
import matplotlib.animation as animation
import psutil

# Parameters
x_len = 200         # Number of points to display
y_range = [0,100]  # Range of possible Y values to display


fig = plt.figure()
ax1 = plt.subplot2grid((2, 2), (0, 0))
ax2 = plt.subplot2grid((2, 2), (0, 1))

# plt.show()
buffer = list(range(0, 200))
ys1 = [0]*x_len
ax1.set_ylim(y_range)





line1, = ax1.plot(buffer, ys1) # Create a blank line. We will update the line in animate


# This function is called periodically from FuncAnimation
def animate(i,ys):
   
    ys.append(psutil.cpu_percent()) # Add y to list
    ys = ys[-x_len:] # Limit y list to set number of items
    line1.set_ydata(ys) # Update line with new Y values


    
    return line1,

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig,
    animate,
    fargs=(ys1,),
    interval=50,
    blit=True)
plt.show()


