std_msgs/Time Time

## STATES WRT ORIGIN
geometry_msgs/Pose Pose_B_O
geometry_msgs/Twist Twist_B_O
geometry_msgs/Accel Accel_B_O
geometry_msgs/Vector3 Eul_B_O
float64 Accel_B_O_Mag

## STATES WRT PLANE
geometry_msgs/Pose Pose_P_B
geometry_msgs/Twist Twist_B_P
geometry_msgs/Vector3 Eul_P_B

float64 Vel_mag_B_P
float64 Vel_angle_B_P
float64 D_perp
float64 D_perp_min
float64 D_perp_CR



## OPTICAL FLOW STATES
geometry_msgs/Vector3 Optical_Flow
geometry_msgs/Vector3 Optical_Flow_Cam
float64 Tau_CR

## STATE SETPOINTS
geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

## CONTROL ACTIONS
float64[4] FM
float64[4] MotorThrusts
uint32[4] MS_PWM

## POLICY ACTIONS
float64 Policy_Trg_Action
float64 Policy_Rot_Action

