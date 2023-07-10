## CONTROLLER STATES
geometry_msgs/Pose Pose
geometry_msgs/Twist Twist


geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

float64[4] FM
float64[4] MotorThrusts
uint32[4] MS_PWM

float64 Policy_Trg_Action
float64 Policy_Flip_Action

## PLANE RELATIVE STATES
float64 D_perp
float64 V_perp
float64 V_tx
float64 V_ty


## OPTICAL FLOW STATES
float64 Tau
float64 Theta_x
float64 Theta_y
float64 Theta_z


float64 Tau_est
float64 Theta_x_est
float64 Theta_y_est
float64 Theta_z_est


# STATES AT FLIP TRIGGER
bool flip_flag
geometry_msgs/Pose Pose_tr
geometry_msgs/Twist Twist_tr
float64 Tau_tr
float64 Theta_x_tr
float64 Theta_y_tr
float64 Theta_z_tr

float64 D_perp_tr

float64[4] FM_flip

float64 Policy_Flip_tr
float64 Policy_Action_tr


