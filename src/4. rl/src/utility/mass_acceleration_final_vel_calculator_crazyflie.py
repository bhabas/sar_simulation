#Estimate weight, acceleration, etc. for crazyflie

#Define Constants
base_mass_g= 29.5 #crazyflie base vehicle mass (no payload) (grams)
distance= 1.1 #vertical acceleration distance (meters)
v_z_f=2.5 #required vertical velocity to permit successful landing (m/s)
g=9.81 #acceleration due to gravity (m/s)
max_thrust_g=38 #(maximum thrust of quadrotor in grams)
#calculated constants
thrust_n= max_thrust_g/1000*g#vehicle thrust (newtons)

#calculate required acceleration to achieve desired velocity (assume constant accel)
a_z_req= v_z_f**2/(2*distance) #required acceleration in z direction (m/s)
mass_max= thrust_n/(a_z_req+g) #maximum mass to achieve required acceleration

#print results
print ("Max Total Mass " + "{:.2f}".format(mass_max*1000) +" grams")
print ("Max Payload Mass " + "{:.2f}".format(mass_max*1000-base_mass_g) +" grams")

