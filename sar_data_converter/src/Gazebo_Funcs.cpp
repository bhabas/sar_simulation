#include "Gazebo_Funcs.h"

/**
 * @brief Function called that will service call the sticky foot plugin and join pad to landing surface
 *
 */
void SAR_DataConverter::activateStickyFeet()
{

    sar_msgs::Activate_Sticky_Pads srv;
    srv.request.Sticky_Flag = Sticky_Flag;

    ros::service::call("/SAR_Internal/Sticky_Leg_1", srv);
    ros::service::call("/SAR_Internal/Sticky_Leg_2", srv);
    ros::service::call("/SAR_Internal/Sticky_Leg_3", srv);
    ros::service::call("/SAR_Internal/Sticky_Leg_4", srv);
}

/**
 * @brief Updates if sticky pads attach to landing surface. Also calculates total number of pads attached
 *
 * @param msg sar_msgs::Sticky_Pad_Connect
 */
void SAR_DataConverter::Pad_Connections_Callback(const sar_msgs::Sticky_Pad_Connect &msg)
{

    if (msg.Pad1_Contact == 1)
        Pad1_Contact = 1;
    if (msg.Pad2_Contact == 1)
        Pad2_Contact = 1;
    if (msg.Pad3_Contact == 1)
        Pad3_Contact = 1;
    if (msg.Pad4_Contact == 1)
        Pad4_Contact = 1;
    Pad_Connect_Sum = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
}

/**
 * @brief Updates the position and orientation of the landing surface in Gazebo.
 *
 * @param Pos_x X-position of surface
 * @param Pos_y Y-position of surface
 * @param Pos_z Z-position of surface
 * @param Plane_Angle 0 deg -> Horizontal Ground Surface | 90 deg -> Vertical Wall | 180 deg -> Ceiling Surface
 */
void SAR_DataConverter::Update_Landing_Surface_Pose(float Pos_x, float Pos_y, float Pos_z, float Plane_Angle)
{

    sar_msgs::Surface_Settings srv;

    srv.request.Pos.x = Pos_x;
    srv.request.Pos.y = Pos_y;
    srv.request.Pos.z = Pos_z;
    srv.request.Plane_Angle = Plane_Angle;

    Landing_Surface_Pose_Client.call(srv);
}

/**
 * @brief Records if body of SAR body collides with landing surface so as to serve as a negative aspect to reward
 *
 * @param msg gazebo_msgs::ContactsState
 */
void SAR_DataConverter::Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg)
{

    // CYCLE THROUGH LIST OF CONTACT MESSAGES
    for (int i = 0; i < msg.states.size(); i++)
    {
        // IF CONTACT MSG MATCHES BODY COLLISION STR THEN TURN ON BODY_CONTACT_FLAG
        if (BodyContact_flag == false && strcmp(msg.states[i].collision1_name.c_str(), BodyCollision_str.c_str()) == 0)
        {
            BodyContact_flag = true;
        }

        if (impact_flag == false)
        {
            // LOCK IN STATE DATA WHEN IMPACT DETECTED
            impact_flag = true;

            // RECORD IMPACT STATE DATA FROM END OF CIRCULAR BUFFER WHEN IMPACT FLAGGED
            Time_impact = ros::Time::now();
            Pose_impact = Pose_impact_buff.front();
            Twist_impact = Twist_impact_buff.front();

            // PROCESS EULER ANGLES AT TIME OF IMPACT
            float quat_impact[4] = {
                (float)Pose_impact.orientation.x,
                (float)Pose_impact.orientation.y,
                (float)Pose_impact.orientation.z,
                (float)Pose_impact.orientation.w};
            float eul_impact[3];
            quat2euler(quat_impact, eul_impact);
            Eul_impact.x = eul_impact[0] * 180 / M_PI;
            Eul_impact.y = eul_impact[1] * 180 / M_PI;
            Eul_impact.z = eul_impact[2] * 180 / M_PI;
        }
    }
}

/**
 * @brief Records max impact force from SAR colliding with landing surface. It also records
 * impact states like pose, vel, impact time, and euler angles which are used in reward function.
 *
 * @param msg geometry_msgs::WrenchStamped
 */
void SAR_DataConverter::SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    // RECORD MAX FORCE EXPERIENCED
    if (msg->wrench.force.x > impact_force_x)
    {
        impact_force_x = msg->wrench.force.x;
    }
    if (msg->wrench.force.y > impact_force_y)
    {
        impact_force_y = msg->wrench.force.y;
    }
    if (msg->wrench.force.z > impact_force_z)
    {
        impact_force_z = msg->wrench.force.z;
    }

    impact_magnitude = sqrt(pow(msg->wrench.force.x, 2) + pow(msg->wrench.force.y, 2) + pow(msg->wrench.force.z, 2));
}


/**
 * @brief Updates simulation speed based on how close SAR is to landing surface. Sim becomes unstable
 * and accurate impact data is lost if collision happens too fast.
 */
void SAR_DataConverter::checkSlowdown()
{
    // SIMULATION SLOWDOWN
    if (LANDING_SLOWDOWN_FLAG == true)
    {

        // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
        if (D_perp <= 0.5 && SLOWDOWN_TYPE == 0)
        {

            // SAR_DataConverter::adjustSimSpeed(SIM_SLOWDOWN_SPEED);
            SLOWDOWN_TYPE = 1;
        }

        // IF IMPACTED CEILING OR FALLING AWAY, INCREASE SIM SPEED TO DEFAULT
        if (impact_flag == true && SLOWDOWN_TYPE == 1)
        {
            // SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2; // (Don't call adjustSimSpeed more than once)
        }
        else if (Twist.linear.z <= -0.5 && SLOWDOWN_TYPE == 1)
        {
            // SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2;
        }
    }
}
