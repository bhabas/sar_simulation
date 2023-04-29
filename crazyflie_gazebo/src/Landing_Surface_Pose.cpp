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

        Surface_Model_Ptr = World_Ptr->ModelByName("Standard_Plane");
        Surface_Model_Ptr->CreateJoint("Landing_Surface_Joint","fixed",Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Surface_Link"));
        Joint_Ptr = Surface_Model_Ptr->GetJoint("Landing_Surface_Joint");
        Joint_Ptr->SetProvideFeedback(true);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Landing_Surface_Pose::OnUpdate, this));
        printf("\n\n");

    }

    void Landing_Surface_Pose::OnUpdate()
    {
        physics::JointWrench wrench;
        ignition::math::Vector3d torque;
        ignition::math::Vector3d force;

        // FIXME: Should include options for diferent frames and measure directions
        // E.g: https://bitbucket.org/osrf/gazebo/raw/default/gazebo/sensors/ForceTorqueSensor.hh
        // Get force torque at the joint
        // The wrench is reported in the CHILD <frame>
        // The <measure_direction> is child_to_parent
        

        if (Joint_Ptr != NULL)
        {
            // REMOVE JOINT BETWEEN LANDING SURFACE AND WORLD

            wrench = Joint_Ptr->GetForceTorque(0);
            force = wrench.body2Force;
            torque = wrench.body2Torque;

            printf("Vals %.3f\n",force.Z());
 
        }

        


    }


    bool Landing_Surface_Pose::Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res)
    {
        
        // Surface_Model_Ptr = World_Ptr->ModelByName(req.model_state.model_name);
        // Joint_Ptr = Surface_Model_Ptr->GetJoint("Landing_Surface_Joint");

        // if (Joint_Ptr != NULL)
        // {
        //     // REMOVE JOINT BETWEEN LANDING SURFACE AND WORLD
        //     gzmsg << "Removing Surface-to-World Joint\n";
        //     Surface_Model_Ptr->RemoveJoint("Landing_Surface_Joint");
        // }


        // // UPDATE PLANE POSITION AND ORIENTATION
        // Pos_0.Set(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
        // Quat_0.Set(req.model_state.pose.orientation.w,
        //             req.model_state.pose.orientation.x,
        //             req.model_state.pose.orientation.y,
        //             req.model_state.pose.orientation.z);

        // Pose_0.Set(Pos_0,Quat_0);
        // Surface_Model_Ptr->SetWorldPose(Pose_0);

        // // CREATE JOINT BETWEEN LANDING SURFACE AND WORLD
        // gzmsg << "Creating Surface-to-World Joint\n";
        // Surface_Model_Ptr->CreateJoint("Landing_Surface_Joint","fixed",Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Surface_Link"));

        return true;
    }

    GZ_REGISTER_WORLD_PLUGIN(Landing_Surface_Pose);
}