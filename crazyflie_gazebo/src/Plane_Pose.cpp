#include <iostream>
#include <Plane_Pose.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void Plane_Pose::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";
        world_ptr = _parent;

        _parent->InsertModelFile("model://Standard_Plane");


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
       
        

        Origin_Model_Ptr = world_ptr->ModelByIndex(0);
        Surface_Model_Ptr = world_ptr->ModelByIndex(1);

        std::cout << Origin_Model_Ptr->GetName() << std::endl;
        std::cout << Surface_Model_Ptr->GetName() << std::endl;

  
        if (req.Freq.x == 0)
        {
            gzmsg << "Creating Surface->World Joint\n";
            Surface_Model_Ptr->CreateJoint("plane_joint","fixed",Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Surface_Link"));
        }
        else
        {
            gzmsg << "Removing Surface->World Joint\n";
            Surface_Model_Ptr->RemoveJoint("plane_joint");
        }
        
        
        return true;
    }

    GZ_REGISTER_WORLD_PLUGIN(Plane_Pose);
}