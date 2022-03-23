import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

BASEPATH = "crazyflie_projects/Featureless_TTC/"

## DEFINE SURFACE DIMENSIONS
CEILING_WIDTH = 1.0     # [m]
CEILING_LENGTH = 1.0   # [m]

## DEFINE SQUARE SIZE
sqr_size = 0.05 # [m]



pxls_per_square = 30 # [pixels]
num_sqr_Wide = int(CEILING_WIDTH/sqr_size)
num_sqr_Long = int(CEILING_LENGTH/sqr_size)

## CREATE BASIC CHECKERBOARD
print("Checkerboard pattern:")
arr = np.zeros((num_sqr_Wide,num_sqr_Long),dtype=int)
arr[1::2,:] = 1
# arr[::2,1::2] = 1

## INCREASE PIXEL COUNT IN CHECKERBOARD
arr = np.repeat(arr,pxls_per_square,axis=1)
arr = np.repeat(arr,pxls_per_square,axis=0)

## PLOT CHECKERBOARD IMAGE
plt.imshow(arr,vmin=0, vmax=1, cmap=cm.gray)
plt.show()

## SAVE CHECKERBOARD IMAGE
plt.imsave(
    f'{BASEPATH}/Surface_Patterns/Checkerboard__{sqr_size:.3f}m_{CEILING_WIDTH:.1f}m_x_{CEILING_LENGTH:.1f}m.png', 
    arr, 
    cmap=cm.Greys
)
