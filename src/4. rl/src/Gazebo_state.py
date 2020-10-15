import os
from scipy.spatial.transform import Rotation


x = 0
y = 0
z = 1

vx = 0
vy = 0
vz = 0

omega_x = 0
omega_y = 0
omega_z = 0

r = Rotation.from_euler('y',90,degrees=True)
qx,qy,qz,qw = r.as_quat().tolist()



str = """rosservice call /gazebo/set_model_state 
'{model_state: { model_name: crazyflie_landing_gears, 
pose: { position: { x: %d, y: %d ,z: %d }, 
orientation: {x: %d, y: %d, z: %d, w: %d } }, 
twist: { linear: {x: %d , y: %d ,z: %d } , 
angular: { x: %d , y: %d, z: %d } } , 
reference_frame: world } }'""" %(x,y,z,vx,vy,vz,qx,qy,qz,qw,omega_x,omega_y,omega_z)


os.system(str)