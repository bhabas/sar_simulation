#include <iostream>
#include <Landing_Surface_Pose.h>

/*
The purpose of this plugin is to move the landing surface to the desired position and orientation.
It will then create a fixed joint that will lock it into place. If the surface needs to be relocated,
it will break the joint, move surface to a new location, and then recreate the joint.
*/

namespace gazebo
{

    void Landing_Surface_Pose::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";
        World_Ptr = _parent;
        Origin_Model_Ptr = World_Ptr->ModelByName("World_Origin");


        // LINK COMMAND SERVICE TO CALLBACK
        CMD_Service = nh.advertiseService("/Landing_Surface_Pose", &Landing_Surface_Pose::Service_Callback, this);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Landing_Surface_Pose::OnUpdate, this));
        printf("\n\n");

    }

    void Landing_Surface_Pose::OnUpdate()
    {
        /* DO NOTHING */
    }


    bool Landing_Surface_Pose::Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res)
    {
        
        Surface_Model_Ptr = World_Ptr->ModelByName(req.Model_Name);
        Joint_Ptr = Surface_Model_Ptr->GetJoint("Landing_Surface_Joint");

        if (Joint_Ptr != NULL)
        {
            // REMOVE JOINT BETWEEN LANDING SURFACE AND WORLD
            gzmsg << "Removing Surface-to-World Joint\n";
            Surface_Model_Ptr->RemoveJoint("Landing_Surface_Joint");
        }


        // UPDATE PLANE POSITION AND ORIENTATION
        Pos_0.Set(req.Pos_0.x,req.Pos_0.y,req.Pos_0.z);
        Eul_0.Set(req.Eul_0.x,req.Eul_0.y,req.Eul_0.z);
        Eul_0 *= M_PI/180; // Convert to radians

        Pose_0.Set(Pos_0,Eul_0);
        Surface_Model_Ptr->SetWorldPose(Pose_0);

        // CREATE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Creating Surface-to-World Joint\n";
        Surface_Model_Ptr->CreateJoint("Landing_Surface_Joint","fixed",Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Surface_Link"));

        return true;
    }

    GZ_REGISTER_WORLD_PLUGIN(Landing_Surface_Pose);
}