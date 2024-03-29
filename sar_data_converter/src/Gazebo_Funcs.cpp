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
    Pad_Connections = Pad1_Contact + Pad2_Contact + Pad3_Contact + Pad4_Contact;
}

/**
 * @brief Updates the position and orientation of the landing surface in Gazebo.
 *
 * @param Pos_x X-position of surface
 * @param Pos_y Y-position of surface
 * @param Pos_z Z-position of surface
 * @param Plane_Angle_deg 0 deg -> Horizontal Ground Surface | 90 deg -> Vertical Wall | 180 deg -> Ceiling Surface
 */
void SAR_DataConverter::setLandingSurfacePose(float Pos_x, float Pos_y, float Pos_z, float Plane_Angle_deg)
{

    sar_msgs::Surface_Params srv;

    srv.request.Pos.x = Pos_x;
    srv.request.Pos.y = Pos_y;
    srv.request.Pos.z = Pos_z;
    srv.request.Plane_Angle = Plane_Angle_deg;

    Landing_Surface_Pose_Client.call(srv);
}

/**
 * @brief Records if SAR body collides with landing surface so as to serve as a negative aspect to reward
 *
 * @param msg gazebo_msgs::ContactsState
 */
void SAR_DataConverter::Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg)
{
    // CYCLE THROUGH LIST OF CONTACT MESSAGES
    for (int i = 0; i < msg.states.size(); i++)
    {
        // IF CONTACT MSG MATCHES BODY COLLISION STR THEN TURN ON BODY_CONTACT_FLAG
        if (msg.states[i].collision1_name.find(BodyCollision_str) != std::string::npos && BodyContact_Flag == false)
        {
            BodyContact_Flag = true;
            // std::cout << "Body Contact: " << msg.states[i].collision1_name << std::endl;
        }

        // IF CONTACT MSG MATCHES LEG COLLISION STR THEN TURN ON LEG_CONTACT_FLAG
        if (msg.states[i].collision1_name.find(LegCollision_str) != std::string::npos && (!ForelegContact_Flag && !HindlegContact_Flag))
        {
            char lastChar = msg.states[i].collision1_name.back();
            if (lastChar == '1' || lastChar == '4') {
                ForelegContact_Flag = true;
            }
            else if (lastChar == '2' || lastChar == '3') {
                HindlegContact_Flag = true;
            }
            
            // std::cout << "Leg Contact: " << msg.states[i].collision1_name << std::endl;
        }

        // LOCK IN STATE DATA WHEN INITIAL IMPACT DETECTED
        if (Impact_Flag_Ext == false && (BodyContact_Flag == true || ForelegContact_Flag == true || HindlegContact_Flag == true))
        {
            Impact_Flag_Ext = true;

            // RECORD IMPACT STATE DATA FROM END OF CIRCULAR BUFFER WHEN IMPACT FLAGGED
            Time_impact_Ext = ros::Time::now();
            Pose_B_O_impact_Ext = Pose_B_O_impact_buff.front();
            Eul_B_O_impact_Ext = Eul_B_O_impact_buff.front();
            Twist_B_P_impact_Ext = Twist_P_B_impact_buff.front();
            Eul_P_B_impact_Ext = Eul_P_B_impact_buff.front();
            Rot_Sum_impact_Ext = Rot_Sum;


        }

    }
}

/**
 * @brief Records max impact force from SAR colliding with landing surface.
 *
 * @param msg geometry_msgs::WrenchStamped
 */
void SAR_DataConverter::SurfaceFT_Sensor_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    if (sqrt(pow(msg->wrench.force.x, 2) + pow(msg->wrench.force.y, 2) + pow(msg->wrench.force.z, 2)) > Impact_Magnitude)
    {
        // RECORD MAX FORCE EXPERIENCED
        Impact_Magnitude = sqrt(pow(msg->wrench.force.x, 2) + pow(msg->wrench.force.y, 2) + pow(msg->wrench.force.z, 2));
        Force_Impact_x = msg->wrench.force.x;
        Force_Impact_y = msg->wrench.force.y;
        Force_Impact_z = msg->wrench.force.z;
    }
    
}

/**
 * @brief Updates simulation real time factor (RTF) to the provided value
 *
 * @param RTF Simulation speed adjustment. (0.5 = 0.5 x Real Time Speed)
 */
void SAR_DataConverter::adjustSimSpeed(float RTF)
{
    gazebo_msgs::SetPhysicsProperties srv;
    float time_step = 0.001;
    srv.request.time_step = time_step;
    srv.request.max_update_rate = (int)(RTF / time_step);

    geometry_msgs::Vector3 gravity_vec;
    gravity_vec.x = 0.0;
    gravity_vec.y = 0.0;
    gravity_vec.z = -9.8066;
    srv.request.gravity = gravity_vec;

    gazebo_msgs::ODEPhysics ode_config;
    ode_config.auto_disable_bodies = false;
    ode_config.sor_pgs_precon_iters = 0;
    ode_config.sor_pgs_iters = 50;
    ode_config.sor_pgs_w = 1.3;
    ode_config.sor_pgs_rms_error_tol = 0.0;
    ode_config.contact_surface_layer = 0.001;
    ode_config.contact_max_correcting_vel = 0.0;
    ode_config.cfm = 0.0;
    ode_config.erp = 0.2;
    ode_config.max_contacts = 20;

    srv.request.ode_config = ode_config;

    GZ_SimSpeed_Client.call(srv);
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

        // WHEN CLOSE TO THE LANDING SURFACE REDUCE SIM SPEED
        if (D_perp <= 0.5 && SLOWDOWN_TYPE == 0)
        {
            // std::cout << D_perp << std::endl;
            SAR_DataConverter::adjustSimSpeed(SIM_SLOWDOWN_SPEED);
            SLOWDOWN_TYPE = 1;
        }

        // IF IMPACTED LANDING SURFACE OR FALLING AWAY, INCREASE SIM SPEED TO DEFAULT
        if (Impact_Flag_Ext == true && SLOWDOWN_TYPE == 1)
        {
            SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2; // (Don't call adjustSimSpeed more than once)
        }
        else if (Twist_B_P.linear.z <= -0.5 && SLOWDOWN_TYPE == 1)
        {
            SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2;
        }
    }
}
