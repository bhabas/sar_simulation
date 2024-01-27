# =========================
## CONTROLLER STATES
# =========================
geometry_msgs/Pose Pose_B_O
geometry_msgs/Twist Twist_B_O
geometry_msgs/Accel Accel_B_O
float64 Accel_B_O_Mag

geometry_msgs/Pose Pose_P_B
geometry_msgs/Twist Twist_B_P
float64 Vel_mag_B_P
float64 Vel_angle_B_P
float64 D_perp

## OPTICAL FLOW STATES
geometry_msgs/Vector3 Optical_Flow
geometry_msgs/Vector3 Optical_Flow_Cam

geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

float64[4] FM
float64[4] MotorThrusts
uint32[4] MS_PWM

float64 Policy_Trg_Action
float64 Policy_Rot_Action


# ==========================
#  STATES AT POLICY TRIGGER
# ==========================
bool Trg_Flag

geometry_msgs/Pose Pose_B_O_trg
geometry_msgs/Twist Twist_B_O_trg

geometry_msgs/Pose Pose_P_B_trg
geometry_msgs/Twist Twist_B_P_trg

geometry_msgs/Vector3 Optical_Flow_trg
float64 Policy_Trg_Action_trg
float64 Policy_Rot_Action_trg

# =========================
#      STATES AT IMPACT
# =========================

bool Impact_Flag_OB

geometry_msgs/Pose Pose_B_O_impact_OB
geometry_msgs/Twist Twist_B_P_impact_OB
float64 Accel_B_O_Mag_impact_OB





