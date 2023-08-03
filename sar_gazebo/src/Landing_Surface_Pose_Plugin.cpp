
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

        // LOAD MODEL AND WORLD POINTERS
        Surface_Model_Ptr = _parent;
        Surface_Link_Ptr = Surface_Model_Ptr->GetLink("Plane_Base_Model::Surface_Link");

        World_Ptr = Surface_Model_Ptr->GetWorld();
        World_Origin_Model_Ptr = World_Ptr->ModelByName("World_Origin");
        World_Origin_Link_Ptr = World_Origin_Model_Ptr->GetLink("Origin_Link");

        // LOAD PARAMS FROM SDF
        Joint_Name = _sdf->GetElement("jointName")->Get<std::string>();
        ForceTorque_Topic = _sdf->GetElement("topicName")->Get<std::string>();

        // CREATE INITIAL JOINT TO WORLD
        Surface_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Link_Ptr,Surface_Link_Ptr);
        Joint_Ptr = Surface_Model_Ptr->GetJoint(Joint_Name);
        Joint_Ptr->SetProvideFeedback(true);

        // LOAD INITIAL POSE FROM SDF
        ros::param::get("/PLANE_SETTINGS/Pos_X",Pos_X);
        ros::param::get("/PLANE_SETTINGS/Pos_Y",Pos_Y);
        ros::param::get("/PLANE_SETTINGS/Pos_Z",Pos_Z);
        ros::param::get("/PLANE_SETTINGS/Plane_Angle",Plane_Angle);


        // UPDATE POSE
        Landing_Surface_Pose::Update_Pose();



        // LINK COMMAND SERVICE TO CALLBACK
        Pose_Update_Service = nh.advertiseService("/ENV/Landing_Surface_Pose", &Landing_Surface_Pose::Service_Callback, this);


        
        
        // // START UPDATE AND PUBLISHER THREADS
        // ForceTorque_Pub = nh.advertise<geometry_msgs::WrenchStamped>(ForceTorque_Topic,1);
        // ForceTorque_Publisher_Thread = std::thread(&Landing_Surface_Pose::ForceTorque_Publisher, this);
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Landing_Surface_Pose::OnUpdate, this));

        printf("\n\n");

    }

    void Landing_Surface_Pose::Update_Pose()
    {

        // FLAG THAT JOINT IS BEING UPDATED
        UpdatingJoint = true;

        // REMOVE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Removing Surface-to-World Joint\n";
        Surface_Model_Ptr->RemoveJoint(Joint_Name);


        // UPDATE PLANE POSITION AND ORIENTATION
        Pose.Set(Pos_X,Pos_Y,Pos_Z, 0.0,-Plane_Angle*M_PI/180.0, 0.0);
        Surface_Model_Ptr->SetWorldPose(Pose);


         // CREATE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Creating Surface-to-World Joint\n";
        Surface_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Link_Ptr,Surface_Link_Ptr);
        Joint_Ptr = Surface_Model_Ptr->GetJoint(Joint_Name);
        Joint_Ptr->SetProvideFeedback(true);

        // TURN OFF UPDATING FLAG
        UpdatingJoint = false;

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
            double mass = Surface_Link_Ptr->GetInertial()->Mass();
            ignition::math::Vector3d gravity_world_frame = World_Ptr->Gravity();

            ignition::math::Pose3d pose_difference = Surface_Link_Ptr->WorldCoGPose();
            ignition::math::Vector3d gravity_link_frame = -pose_difference.Rot().Inverse().RotateVector(gravity_world_frame);





            ignition::math::Vector3d force_due_to_gravity = mass * gravity_link_frame;
            ignition::math::Vector3d torque_due_to_gravity = Surface_Link_Ptr->GetInertial()->CoG() * force_due_to_gravity;

         

            ignition::math::Vector3d net_force = Joint_Ptr->GetForceTorque(0).body2Force;
            ignition::math::Vector3d net_torque = Joint_Ptr->GetForceTorque(0).body2Torque;

            

            ignition::math::Vector3d force_excluding_gravity = net_force + gravity_link_frame;
            ignition::math::Vector3d torque_excluding_gravity = net_torque + torque_due_to_gravity;

            

            std::cout << gravity_link_frame << std::endl;
            std::cout << net_force << std::endl;
            std::cout << net_torque << std::endl;
            std::cout << force_excluding_gravity << std::endl;
            std::cout << torque_excluding_gravity << std::endl;
    
            std::cout << std::endl;

            




            Force_Vec = Joint_Ptr->GetForceTorque(0).body2Force;
            Torque_Vec = Joint_Ptr->GetForceTorque(0).body2Torque;

        }

    }


    bool Landing_Surface_Pose::Service_Callback(sar_msgs::Surface_Settings::Request &req, sar_msgs::Surface_Settings::Response &res)
    {
        // UPDATING SURFACE POSE
        Pos_X = req.Pos.x;
        Pos_Y = req.Pos.y;
        Pos_Z = req.Pos.z;
        Plane_Angle = req.Plane_Angle;

        Landing_Surface_Pose::Update_Pose();

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Landing_Surface_Pose);
}