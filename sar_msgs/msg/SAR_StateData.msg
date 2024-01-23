Header header
geometry_msgs/Pose Pose
geometry_msgs/Twist Twist
geometry_msgs/Accel Accel
geometry_msgs/Vector3 Eul

## OPTICAL FLOW GROUND TRUTH
float64 Tau
float64 Theta_x
float64 Theta_y

## OPTICAL FLOW ESTIMATES
float64 Tau_est
float64 Theta_x_est
float64 Theta_y_est

## PLANE RELATIVE STATES
float64 D_perp
float64 V_perp
float64 V_tx
float64 V_ty

float64 Vel_rel
float64 Phi_rel

float64[4] FM
float64[4] MotorThrusts
float64[4] MS_PWM

float64 Policy_Trg_Action
float64 Policy_Rot_Action

## CONTROLLER SETPOINTS
geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

