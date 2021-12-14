import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from skimage import measure
from skimage.draw import ellipsoid


# # Generate a level set about zero of two identical ellipsoids in 3D
# ellip_base = ellipsoid(6, 10, 16, levelset=True)
# ellip_double = np.concatenate((ellip_base[:-1, ...],
#                                ellip_base[2:, ...]), axis=0)

# # Use marching cubes to obtain the surface mesh of these ellipsoids
# verts, faces, normals, values = measure.marching_cubes(ellip_double, 0)


# Create a data volume (30 x 30 x 30)
X, Y, Z = np.mgrid[:30, :30, :30]
u = (X-15)**2 + (Y-15)**2 + (Z-15)**2 - 8**2

# Extract the 0-isosurface
verts, faces, normals, values = measure.marching_cubes(u, 0)


# Display resulting triangular mesh using Matplotlib. This can also be done
# with mayavi (see skimage.measure.marching_cubes_lewiner docstring).
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

# Fancy indexing: `verts[faces]` to generate a collection of triangles
mesh = Poly3DCollection(verts[faces])
mesh.set_edgecolor('k')
ax.add_collection3d(mesh)

ax.set_xlabel("x-axis: a = 6 per ellipsoid")
ax.set_ylabel("y-axis: b = 10")
ax.set_zlabel("z-axis: c = 16")

ax.set_xlim(0, 24)  # a = 6 (times two for 2nd ellipsoid)
ax.set_ylim(0, 20)  # b = 10
ax.set_zlim(0, 32)  # c = 16

plt.tight_layout()
plt.show()









import numpy as np
# import mcubes

# # Create a data volume (30 x 30 x 30)
# X, Y, Z = np.mgrid[:30, :30, :30]
# u = (X-15)**2 + (Y-15)**2 + (Z-15)**2 - 8**2

# # Extract the 0-isosurface
# vertices, triangles = mcubes.marching_cubes(u, 0)

# # Export the result to sphere.dae
# mcubes.export_mesh(vertices, triangles, "sphere.dae", "MySphere")