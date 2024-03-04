
bool Trg_Flag
std_msgs/Time Time_trg


## STATES WRT ORIGIN
geometry_msgs/Pose Pose_B_O_trg
geometry_msgs/Twist Twist_B_O_trg
geometry_msgs/Vector3 Eul_B_O_trg
float64 Vel_mag_B_O_trg
float64 Vel_angle_B_O_trg

## STATES WRT PLANE
geometry_msgs/Pose Pose_P_B_trg
geometry_msgs/Twist Twist_B_P_trg
geometry_msgs/Vector3 Eul_P_B_trg
float64 Vel_mag_B_P_trg
float64 Vel_angle_B_P_trg
float64 D_perp_trg
float64 D_perp_CR_trg

## OPTICAL FLOW STATES
geometry_msgs/Vector3 Optical_Flow_trg
float64 Tau_CR_trg

## POLICY ACTIONS
float64 Policy_Trg_Action_trg
float64 Policy_Rot_Action_trg