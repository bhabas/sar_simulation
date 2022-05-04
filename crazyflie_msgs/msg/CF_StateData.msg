Header header
geometry_msgs/Pose Pose
geometry_msgs/Twist Twist
geometry_msgs/Vector3 Eul

## OPTICAL FLOW GROUND TRUTH
float64 Tau
float64 OFx
float64 OFy
float64 D_ceil

## OPTICAL FLOW ESTIMATES
float64 Tau_est
float64 OFx_est
float64 OFy_est

float64[4] FM
float64[4] MotorThrusts
float64[4] MS_PWM

float64 NN_flip
float64 NN_policy

## CONTROLLER SETPOINTS
geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

