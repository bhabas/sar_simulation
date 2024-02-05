bool Impact_Flag

## ONBOARD IMPACT DETECTION
bool Impact_Flag_OB
std_msgs/Time Time_impact_OB
geometry_msgs/Pose Pose_B_O_impact_OB
geometry_msgs/Vector3 Eul_B_O_impact_OB

geometry_msgs/Twist Twist_B_P_impact_OB
geometry_msgs/Vector3 Eul_P_B_impact_OB
float64 Accel_B_O_Mag_impact_OB

## EXTERNAL IMPACT DETECTION
bool Impact_Flag_Ext
std_msgs/Time Time_impact_Ext

bool BodyContact_Flag
bool ForelegContact_Flag
bool HindlegContact_Flag

geometry_msgs/Pose Pose_B_O_impact_Ext
geometry_msgs/Vector3 Eul_B_O_impact_Ext

geometry_msgs/Twist Twist_B_P_impact_Ext
geometry_msgs/Vector3 Eul_P_B_impact_Ext
float64 Rot_Sum_impact_Ext

## IMPACT FORCES
geometry_msgs/Vector3 Force_impact
float64 Impact_Magnitude

## STICKY PAD CONTACTS
uint8 Pad_Connections
uint8 Pad1_Contact
uint8 Pad2_Contact
uint8 Pad3_Contact
uint8 Pad4_Contact