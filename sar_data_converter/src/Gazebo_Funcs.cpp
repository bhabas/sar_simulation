#include "Gazebo_Funcs.h"

void SAR_DataConverter::activateStickyFeet()
{
    if(GZ_Model_Name != "crazyflie_Base_Model")
    {
        crazyflie_msgs::activateSticky srv;
        srv.request.stickyFlag = Sticky_Flag;

        ros::service::call("/activate_Sticky_Pad_1", srv);
        ros::service::call("/activate_Sticky_Pad_2", srv);
        ros::service::call("/activate_Sticky_Pad_3", srv);
        ros::service::call("/activate_Sticky_Pad_4", srv);
    }
    
}

void SAR_DataConverter::Update_Landing_Surface_Pose(float Pos_x, float Pos_y, float Pos_z, float Plane_Angle)
{

    // float eul[3] = {0.0,0.0f,Plane_Angle*M_PI/180.0f};
    float quat[4] = {0.0f,0.0f,0.0f,1.0f};
    // euler2quat(eul,quat);

    gazebo_msgs::SetModelState srv;

    srv.request.model_state.model_name = Plane_Model_Name;

    srv.request.model_state.pose.position.x = Pos_x;
    srv.request.model_state.pose.position.y = Pos_y;
    srv.request.model_state.pose.position.z = Pos_z;

    srv.request.model_state.pose.orientation.x = quat[0];
    srv.request.model_state.pose.orientation.y = quat[1];
    srv.request.model_state.pose.orientation.z = quat[2];
    srv.request.model_state.pose.orientation.w = quat[3];

    Landing_Surface_Pose_Client.call(srv);

}

// CHANGES REAL TIME FACTOR FOR THE SIMULATION (LOWER = SLOWER)
void SAR_DataConverter::adjustSimSpeed(float speed_mult)
{
    gazebo_msgs::SetPhysicsProperties srv;
    srv.request.time_step = 0.001;
    srv.request.max_update_rate = (int)(speed_mult/0.001);


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