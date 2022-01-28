import numpy as np
from numpy import cos, pi
from skimage import measure 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x, y, z = pi*np.mgrid[-1:1:31j, -1:1:31j, -1:1:31j]
vol = cos(x) + cos(y) + cos(z)
iso_val=0.0
verts, faces, _, _ = measure.marching_cubes(vol, iso_val, spacing=(0.1, 0.1, 0.1))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(verts[:, 0], verts[:,1], faces, verts[:, 2], cmap='Spectral',
                lw=1)
plt.show()

# from numpy import cos, pi, mgrid
# import pyvista as pv

# x, y, z = pi*mgrid[-1:1:31j, -1:1:31j, -1:1:31j]
# vol = cos(x) + cos(y) + cos(z)
# grid = pv.StructuredGrid(x, y, z)
# grid["vol"] = vol.flatten()
# contours = grid.contour([0])

# pv.set_plot_theme('document')
# p = pv.Plotter()
# p.add_mesh(contours, scalars=contours.points[:, 2], show_scalar_bar=False)
# p.show()
