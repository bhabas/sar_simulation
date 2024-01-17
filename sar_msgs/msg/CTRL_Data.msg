## CONTROLLER STATES
geometry_msgs/Pose Pose
geometry_msgs/Twist Twist
geometry_msgs/Accel Accel
float64 AccMag

geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

float64[4] FM
float64[4] MotorThrusts
uint32[4] MS_PWM

float64 Policy_Trg_Action
float64 Policy_Rot_Action

## PLANE RELATIVE STATES
float64 D_perp
float64 V_perp
float64 V_tx
float64 V_ty


## OPTICAL FLOW STATES
float64 Tau
float64 Theta_x
float64 Theta_y

float64 Tau_est
float64 Theta_x_est
float64 Theta_y_est


# STATES AT FLIP TRIGGER
bool Rot_flag
geometry_msgs/Pose Pose_trg
geometry_msgs/Twist Twist_trg
geometry_msgs/Accel Accel_trg
float64 Tau_trg
float64 Theta_x_trg
float64 Theta_y_trg

float64 D_perp_trg

float64[4] FM_Rot

float64 Policy_Trg_Action_trg
float64 Policy_Rot_Action_trg


