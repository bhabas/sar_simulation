uint32 Tick

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
float64 D_perp_CR

## PLANE STATES
geometry_msgs/Vector3 Plane_Pos
float64 Plane_Angle_deg

## OPTICAL FLOW STATES
geometry_msgs/Vector3 Optical_Flow
geometry_msgs/Vector3 Optical_Flow_Cam
float64 Tau_CR

geometry_msgs/Vector3 x_d
geometry_msgs/Vector3 v_d
geometry_msgs/Vector3 a_d

float64[4] FM
float64[4] MotorThrusts
uint32[4] Motor_CMD

float64[4] NN_Output
float64 a_Trg
float64 a_Rot


# ==========================
#  STATES AT POLICY TRIGGER
# ==========================
bool Trg_Flag

geometry_msgs/Pose Pose_B_O_trg
geometry_msgs/Twist Twist_B_O_trg

geometry_msgs/Pose Pose_P_B_trg
geometry_msgs/Twist Twist_B_P_trg
float64 Vel_mag_B_P_trg
float64 Vel_angle_B_P_trg
float64 D_perp_trg
float64 D_perp_CR_trg

geometry_msgs/Vector3 Optical_Flow_trg
float64 Tau_CR_trg

float64[4] NN_Output_trg
float64 a_Trg_trg
float64 a_Rot_trg

# =========================
#      STATES AT IMPACT
# =========================

bool Impact_Flag_OB

geometry_msgs/Pose Pose_B_O_impact_OB
geometry_msgs/Twist Twist_B_P_impact_OB
float64 Vel_mag_B_P_impact_OB
float64 Vel_angle_B_P_impact_OB
float64 dOmega_B_O_y_impact_OB





