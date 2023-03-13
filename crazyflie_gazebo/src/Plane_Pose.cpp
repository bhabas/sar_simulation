#include <iostream>
#include <Plane_Pose.h>

namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void Plane_Pose::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";
        World_Ptr = _parent;


        Origin_Model_Ptr = World_Ptr->ModelByName("World_Origin");


        
        



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

        
        Surface_Model_Ptr = World_Ptr->ModelByName(req.Model_Name);
        
        // std::cout << Origin_Model_Ptr->GetName() << std::endl;
        // std::cout << Surface_Model_Ptr->GetName() << std::endl;

        
        

        
        if (req.Create_Joint == true)
        {
            gzmsg << "Creating Surface-to-World Joint\n";

            // UPDATE STARTING POSITION AND ORIENTATION
            Pos_0.Set(req.Pos_0.x,req.Pos_0.y,req.Pos_0.z);
            Eul_0.Set(req.Eul_0.x,req.Eul_0.y,req.Eul_0.z);
            Eul_0 *= M_PI/180; // Convert to radians

            Pose_0.Set(Pos_0,Eul_0);
            Surface_Model_Ptr->SetWorldPose(Pose_0);


            Surface_Model_Ptr->CreateJoint("Landing_Surface_Joint","fixed",Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Surface_Link"));
        }
        else
        {
            gzmsg << "Removing Surface-to-World Joint\n";
            Surface_Model_Ptr->RemoveJoint("Landing_Surface_Joint");
        }
        
        
        return true;
    }

    GZ_REGISTER_WORLD_PLUGIN(Plane_Pose);
}