#include <iostream>
#include <Plane_Pose.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void Plane_Pose::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";
        model_ptr = parent;

        // LINK COMMAND SERVICE TO CALLBACK
        CMD_Service = nh.advertiseService("/Plane_Pose", &Plane_Pose::Service_Callback, this);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Plane_Pose::OnUpdate, this));
        printf("\n\n");

    }

    void Plane_Pose::OnUpdate()
    {
        

    }


    bool Plane_Pose::Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res)
    {
       
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Plane_Pose);
}