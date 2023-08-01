
#include <Landing_Surface_Pose_Plugin.h>

/*
The purpose of this plugin is to move the landing surface to the desired position and orientation.
It will then create a fixed joint that will lock it into place. If the surface needs to be relocated,
it will break the joint, move surface to a new location, and then recreate the joint.

As well it will publish the active forces and torques on the landing plane. This way impact forces can
be recorded.
*/

namespace gazebo
{

    void Landing_Surface_Pose::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Gazebo Plane Pose Plugin\n";

        // LOAD PARAMS FROM SDF
        Joint_Name = _sdf->GetElement("jointName")->Get<std::string>();
        ForceTorque_Topic = _sdf->GetElement("topicName")->Get<std::string>();


        // LOAD MODEL AND WORLD POINTERS
        Surface_Model_Ptr = _parent;
        World_Ptr = Surface_Model_Ptr->GetWorld();
        World_Origin_Model_Ptr = World_Ptr->ModelByName("World_Origin");

        // LINK COMMAND SERVICE TO CALLBACK
        Pose_Update_Service = nh.advertiseService("/Landing_Surface_Pose", &Landing_Surface_Pose::Service_Callback, this);


        // CREATE INITIAL JOINT TO WORLD
        Surface_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Plane_Base_Model::Surface_Link"));
        Joint_Ptr = Surface_Model_Ptr->GetJoint(Joint_Name);
        Joint_Ptr->SetProvideFeedback(true);

        
        // START UPDATE AND PUBLISHER THREADS
        ForceTorque_Pub = nh.advertise<geometry_msgs::WrenchStamped>(ForceTorque_Topic,1);
        ForceTorque_Publisher_Thread = std::thread(&Landing_Surface_Pose::ForceTorque_Publisher, this);
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Landing_Surface_Pose::OnUpdate, this));

        printf("\n\n");

    }

    void Landing_Surface_Pose::ForceTorque_Publisher()
    {

        // PUBLISHING LOOP FREQUENCY
        int loopRate = 500;     // [Hz]
        ros::Rate rate(loopRate);

        while(ros::ok)
        {   
            // ADD FORCE AND TORQUE DATA TO MSG
            ForceTorque_msg.wrench.force.x = Force_Vec.X();
            ForceTorque_msg.wrench.force.y = Force_Vec.Y();
            ForceTorque_msg.wrench.force.z = Force_Vec.Z();


            ForceTorque_msg.wrench.torque.x = Torque_Vec.X();
            ForceTorque_msg.wrench.torque.y = Torque_Vec.Y();
            ForceTorque_msg.wrench.torque.z = Torque_Vec.Z();
            ForceTorque_Pub.publish(ForceTorque_msg);

            rate.sleep();
        }

        

    }

    void Landing_Surface_Pose::OnUpdate()
    {
        
        // SUPPRESS FORCE OUTPUT WHILE JOINT IS BEING UPDATED
        if (UpdatingJoint == false)
        {
           
            Force_Vec = Joint_Ptr->GetForceTorque(0).body2Force;
            Torque_Vec = Joint_Ptr->GetForceTorque(0).body2Torque;

        }

    }


    bool Landing_Surface_Pose::Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res)
    {
        // FLAG THAT JOINT IS BEING UPDATED
        UpdatingJoint = true;

        // REMOVE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Removing Surface-to-World Joint\n";
        Surface_Model_Ptr->RemoveJoint(Joint_Name);


        // UPDATE PLANE POSITION AND ORIENTATION
        Pos.Set(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
        Quat.Set(req.model_state.pose.orientation.w,
                    req.model_state.pose.orientation.x,
                    req.model_state.pose.orientation.y,
                    req.model_state.pose.orientation.z);

        Pose.Set(Pos,Quat);
        Surface_Model_Ptr->SetWorldPose(Pose);

        // CREATE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Creating Surface-to-World Joint\n";
        Surface_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Model_Ptr->GetLink("Origin_Link"),Surface_Model_Ptr->GetLink("Plane_Base_Model::Surface_Link"));
        Joint_Ptr = Surface_Model_Ptr->GetJoint(Joint_Name);
        Joint_Ptr->SetProvideFeedback(true);
        
        // TURN OFF UPDATING FLAG
        UpdatingJoint = false;

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Landing_Surface_Pose);
}