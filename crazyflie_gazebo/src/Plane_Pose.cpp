#include <iostream>
#include <Plane_Pose.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void Plane_Pose::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";
        model_ptr = parent;
        world_ptr = model_ptr->GetWorld();

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

        physics::LinkPtr plane_ptr;
        physics::LinkPtr ball_ptr;

        physics::ModelPtr plane_model;
        physics::ModelPtr ball_model;

        // joint_ptr = model_ptr->GetJoint("plane_joint");
        // ball_ptr = joint_ptr->GetParent();
        // plane_ptr = joint_ptr->GetChild();

        // std::cout << plane_ptr->GetName() << std::endl;

        plane_model = world_ptr->ModelByIndex(0);
        ball_model = world_ptr->ModelByIndex(1);

        std::cout << world_ptr->ModelByIndex(1)->GetName() << std::endl;
        plane_model->GetLink("plane");
        // plane_model->CreateJoint("plane_joint","fixed",plane_model->GetLink("plane"),ball_model->GetLink("ball"));

        if (req.Freq.x == 1)
        {
            plane_model->CreateJoint("plane_joint","fixed",plane_model->GetLink("plane"),ball_model->GetLink("ball"));
        }
        else
        {
            plane_model->RemoveJoint("plane_joint");
        }
        
        






        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Plane_Pose);
}