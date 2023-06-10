Header header
geometry_msgs/Pose Pose
geometry_msgs/Twist Twist
geometry_msgs/Vector3 Eul

## OPTICAL FLOW GROUND TRUTH
float64 Tau
float64 Theta_x
float64 Theta_y
float64 D_perp

## OPTICAL FLOW ESTIMATES
float64 Tau_est
float64 Theta_x_est
float64 Theta_y_est

float64[4] FM
float64[4] MotorThrusts
float64[4] MS_PWM

float64 Policy_Flip
float64 Policy_Action

## CONTROLLER SETPOINTS
geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

