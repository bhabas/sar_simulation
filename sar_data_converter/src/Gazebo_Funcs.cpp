#include "Gazebo_Funcs.h"


/**
 * @brief Function called that will service call the sticky foot plugin and join pad to landing surface
 * 
 */
void SAR_DataConverter::activateStickyFeet()
{
    if(GZ_Model_Name != "Crazyflie_Base_Model")
    {
        sar_msgs::Activate_Sticky_Pads srv;
        srv.request.stickyFlag = Sticky_Flag;

        ros::service::call("/activate_Sticky_Pad_1", srv);
        ros::service::call("/activate_Sticky_Pad_2", srv);
        ros::service::call("/activate_Sticky_Pad_3", srv);
        ros::service::call("/activate_Sticky_Pad_4", srv);
    }
    
}


/**
 * @brief Updates if sticky pads attach to landing surface. Also calculates total number of pads attached
 * 
 * @param msg sar_msgs::Sticky_Pad_Connect
 */
void SAR_DataConverter::Pad_Connections_Callback(const sar_msgs::Sticky_Pad_Connect &msg)
{
    
    if(msg.Pad1_Contact == 1) Pad1_Contact = 1;
    if(msg.Pad2_Contact == 1) Pad2_Contact = 1;
    if(msg.Pad3_Contact == 1) Pad3_Contact = 1;
    if(msg.Pad4_Contact == 1) Pad4_Contact = 1;
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

    float eul[3] = {0.0*M_PI/180.0, -(float)(Plane_Angle*M_PI/180.0f), 0.0*M_PI/180.0};
    float quat[4] = {0.0f,0.0f,0.0f,1.0f};
    euler2quat(quat,eul);

    gazebo_msgs::SetModelState srv;

    srv.request.model_state.model_name = Plane_Model;

    srv.request.model_state.pose.position.x = Pos_x;
    srv.request.model_state.pose.position.y = Pos_y;
    srv.request.model_state.pose.position.z = Pos_z;

    srv.request.model_state.pose.orientation.x = quat[0];
    srv.request.model_state.pose.orientation.y = quat[1];
    srv.request.model_state.pose.orientation.z = quat[2];
    srv.request.model_state.pose.orientation.w = quat[3];

    Landing_Surface_Pose_Client.call(srv);

}


/**
 * @brief Records if body of SAR body collides with landing surface so as to serve as a negative aspect to reward
 * 
 * @param msg gazebo_msgs::ContactsState
 */
void SAR_DataConverter::Surface_Contact_Callback(const gazebo_msgs::ContactsState &msg)
{
    // CYCLE THROUGH VECTOR OF CONTACT MESSAGES
    for (int i=0; i<msg.states.size(); i++)
    {
        // IF CONTACT MSG MATCHES BODY COLLISION STR THEN TURN ON BODY_CONTACT_FLAG 
        if(BodyContact_flag == false && strcmp(msg.states[i].collision1_name.c_str(),BodyCollision_str.c_str()) == 0)
        {
            BodyContact_flag = true;
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
    if (msg->wrench.force.x > impact_force_x){
        impact_force_x = msg->wrench.force.x;
    }
    if (msg->wrench.force.y > impact_force_y){
        impact_force_y = msg->wrench.force.y;
    }
    if (msg->wrench.force.z > impact_force_z){
        impact_force_z = msg->wrench.force.z;
    }

    // CHECK IF IMPACT FORCE THRESHOLD HAS BEEN CROSSED
    double impact_thr = 0.1;        // Impact threshold [N]
    impact_magnitude = sqrt(pow(msg->wrench.force.x,2) + pow(msg->wrench.force.y,2) + pow(msg->wrench.force.z,2));

    if (impact_magnitude >= impact_thr && impact_flag == false){ 

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
            (float)Pose_impact.orientation.w
        };
        float eul_impact[3];
        quat2euler(quat_impact,eul_impact);
        Eul_impact.x = eul_impact[0]*180/M_PI;
        Eul_impact.y = eul_impact[1]*180/M_PI;
        Eul_impact.z = eul_impact[2]*180/M_PI;

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
    srv.request.max_update_rate = (int)(RTF/time_step);


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
    if(LANDING_SLOWDOWN_FLAG==true){

        // WHEN CLOSE TO THE CEILING REDUCE SIM SPEED
        if(D_perp<=0.5 && SLOWDOWN_TYPE == 0){
            
            SAR_DataConverter::adjustSimSpeed(SIM_SLOWDOWN_SPEED);
            SLOWDOWN_TYPE = 1;
        }

        // IF IMPACTED CEILING OR FALLING AWAY, INCREASE SIM SPEED TO DEFAULT
        if(impact_flag == true && SLOWDOWN_TYPE == 1)
        {
            SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2; // (Don't call adjustSimSpeed more than once)
        }
        else if(Twist.linear.z <= -0.5 && SLOWDOWN_TYPE == 1){
            SAR_DataConverter::adjustSimSpeed(SIM_SPEED);
            SLOWDOWN_TYPE = 2;
        }
    
    }

}

