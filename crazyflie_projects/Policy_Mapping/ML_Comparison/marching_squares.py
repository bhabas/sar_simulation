
import numpy as np
import matplotlib.pyplot as plt

from skimage import measure


# Construct some test data
x = np.linspace(-np.pi,np.pi,100).reshape(-1,1)
y = np.linspace(-np.pi,np.pi,100).reshape(1,-1)
r = np.sqrt(x**2 + y**2)

# Find contours at a constant value of 0.8
contours = measure.find_contours(r, 0.1)

# Display the image and plot all contours found
fig, ax = plt.subplots()
ax.imshow(r, cmap=plt.cm.gray)

for contour in contours:
    ax.plot(contour[:, 1], contour[:, 0], linewidth=2)

ax.axis('image')
ax.set_xticks([])
ax.set_yticks([])
plt.show()
