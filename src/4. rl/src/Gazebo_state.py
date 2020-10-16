import os
from scipy.spatial.transform import Rotation


# x = 0
# y = 0
# z = 1

# vx = 0
# vy = 0
# vz = 0

# omega_x = 0
# omega_y = 0
# omega_z = 0

# r = Rotation.from_euler('y',90,degrees=True)
# qx,qy,qz,qw = r.as_quat().tolist()



str = """
rosservice call /gazebo/set_model_state '{model_state: { model_name: crazyflie_landing_gears, pose: { position: { x: 0, y: 0 ,z: 0.5 }, orientation: {x: 0, y: 0.1736482, z: 0, w: 0.984808 } }, twist: { linear: {x: 0 , y: 0 ,z: 0 } ,angular: { x: 0 , y: 0, z: 0 } } , reference_frame: world } }'
""" 

os.system(str)