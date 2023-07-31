
#include <Cam_Update_Plugin.h>



namespace gazebo
{

    void Cam_Update_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Camera_Update_Plugin\n";

        // Safety check
        if (_parent->GetLink("Camera") == NULL)
        {
            gzerr << "Camera link not found\n";
            return;
        }

        if (_parent->GetLink("SAR_Body") == NULL)
        {
            gzerr << "SAR_Body link not found\n";
            return;
        }

        // LOAD PARAMS FROM SDF
        Joint_Name = _sdf->GetElement("jointName")->Get<std::string>();


        // LOAD MODEL AND WORLD POINTERS
        Base_Model_Ptr = _parent;
        Camera_Link_Ptr = _parent->GetLink("Camera");
        SAR_Body_Ptr = _parent->GetLink("SAR_Body");

        gzmsg << "Removing Camera-to-Model Joint\n";
        Base_Model_Ptr->RemoveJoint(Joint_Name);
        gzmsg << "Removed Camera-to-Model Joint\n";

        ignition::math::Pose3d relativePose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0); // e.g., 1 meter along x-axis
        Camera_Link_Ptr->SetRelativePose(relativePose);



        // // LINK COMMAND SERVICE TO CALLBACK
        // Pose_Update_Service = nh.advertiseService("/Cam_Update_Plugin", &Cam_Update_Plugin::Service_Callback, this);


        // CREATE INITIAL JOINT TO WORLD
        // Base_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Model_Ptr->GetLink("Origin_Link"),Base_Model_Ptr->GetLink("Surface_Link"));
        // Joint_Ptr = Base_Model_Ptr->GetJoint(Joint_Name);
        // Joint_Ptr->SetProvideFeedback(true);

        
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Cam_Update_Plugin::OnUpdate, this));

        printf("\n\n");

    }

    void Cam_Update_Plugin::ForceTorque_Publisher()
    {

        // // PUBLISHING LOOP FREQUENCY
        // int loopRate = 500;     // [Hz]
        // ros::Rate rate(loopRate);

        // while(ros::ok)
        // {   
        //     // ADD FORCE AND TORQUE DATA TO MSG
        //     ForceTorque_msg.wrench.force.x = Force_Vec.X();
        //     ForceTorque_msg.wrench.force.y = Force_Vec.Y();
        //     ForceTorque_msg.wrench.force.z = Force_Vec.Z();


        //     ForceTorque_msg.wrench.torque.x = Torque_Vec.X();
        //     ForceTorque_msg.wrench.torque.y = Torque_Vec.Y();
        //     ForceTorque_msg.wrench.torque.z = Torque_Vec.Z();
        //     ForceTorque_Pub.publish(ForceTorque_msg);

        //     rate.sleep();
        // }

        

    }

    void Cam_Update_Plugin::OnUpdate()
    {
        
        // // SUPPRESS FORCE OUTPUT WHILE JOINT IS BEING UPDATED
        // if (UpdatingJoint == false)
        // {
           
        //     Force_Vec = Joint_Ptr->GetForceTorque(0).body2Force;
        //     Torque_Vec = Joint_Ptr->GetForceTorque(0).body2Torque;

        // }

    }


    bool Cam_Update_Plugin::Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res)
    {
        // FLAG THAT JOINT IS BEING UPDATED
        UpdatingJoint = true;

        // REMOVE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Removing Camera-to-Model Joint\n";
        Base_Model_Ptr->RemoveJoint(Joint_Name);


        // // UPDATE PLANE POSITION AND ORIENTATION
        // Pos.Set(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
        // Quat.Set(req.model_state.pose.orientation.w,
        //             req.model_state.pose.orientation.x,
        //             req.model_state.pose.orientation.y,
        //             req.model_state.pose.orientation.z);

        // Pose.Set(Pos,Quat);
        // Base_Model_Ptr->SetWorldPose(Pose);

        // // CREATE JOINT BETWEEN LANDING SURFACE AND WORLD
        // gzmsg << "Creating Surface-to-World Joint\n";
        // Base_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Model_Ptr->GetLink("Origin_Link"),Base_Model_Ptr->GetLink("Surface_Link"));
        // Joint_Ptr = Base_Model_Ptr->GetJoint(Joint_Name);
        // Joint_Ptr->SetProvideFeedback(true);
        
        // // TURN OFF UPDATING FLAG
        // UpdatingJoint = false;

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Cam_Update_Plugin);
}