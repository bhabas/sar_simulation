
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
        ros::param::get("/PLANE_SETTINGS/Plane_Angle",Plane_Angle_deg);


        // UPDATE POSE
        Landing_Surface_Pose::Update_Pose();



        // LINK COMMAND SERVICE TO CALLBACK
        Pose_Update_Service = nh.advertiseService("/ENV/Landing_Surface_Pose", &Landing_Surface_Pose::Service_Callback, this);


        // START UPDATE THREAD
        ForceTorque_Pub = nh.advertise<geometry_msgs::WrenchStamped>(ForceTorque_Topic,1);
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
        Pose.Set(Pos_X,Pos_Y,Pos_Z, 0.0,(Plane_Angle_deg)*Deg2Rad, 0.0);
        Surface_Model_Ptr->SetWorldPose(Pose);


         // CREATE JOINT BETWEEN LANDING SURFACE AND WORLD
        gzmsg << "Creating Surface-to-World Joint\n";
        Surface_Model_Ptr->CreateJoint(Joint_Name,"fixed",World_Origin_Link_Ptr,Surface_Link_Ptr);
        Joint_Ptr = Surface_Model_Ptr->GetJoint(Joint_Name);
        Joint_Ptr->SetProvideFeedback(true);

        // TURN OFF UPDATING FLAG
        UpdatingJoint = false;

    }

    void Landing_Surface_Pose::OnUpdate()
    {
        
        // SUPPRESS FORCE OUTPUT WHILE JOINT IS BEING UPDATED
        if (UpdatingJoint == false)
        {
            
            // CALC SURFACE POSE AND JOINT FORCES
            double mass = Surface_Link_Ptr->GetInertial()->Mass();
            ignition::math::Pose3d Surface_Link_Pose = Surface_Link_Ptr->WorldCoGPose();
            ignition::math::Vector3d Surface_Link_Force = Joint_Ptr->GetForceTorque(0).body2Force;

            // CALC GRAVITY FORCES IN WORLD AND LINK FRAMES
            ignition::math::Vector3d Gravity_World_Frame = World_Ptr->Gravity();
            ignition::math::Vector3d Gravity_Link_Frame = Surface_Link_Pose.Rot().Inverse().RotateVector(Gravity_World_Frame);

            // CALC NET FORCE ON SURFACE
            ignition::math::Vector3d Force_Due_To_Gravity = mass * Gravity_Link_Frame;
            ignition::math::Vector3d Force_Excluding_Gravity = Surface_Link_Force - Gravity_Link_Frame;
      
            // ADD FORCE AND TORQUE DATA TO MSG
            ForceTorque_msg.wrench.force.x = Force_Excluding_Gravity.X();
            ForceTorque_msg.wrench.force.y = Force_Excluding_Gravity.Y();
            ForceTorque_msg.wrench.force.z = Force_Excluding_Gravity.Z();

            ForceTorque_Pub.publish(ForceTorque_msg);

        }

    }


    bool Landing_Surface_Pose::Service_Callback(sar_msgs::Surface_Params::Request &req, sar_msgs::Surface_Params::Response &res)
    {
        // UPDATING SURFACE POSE
        Pos_X = req.Pos.x;
        Pos_Y = req.Pos.y;
        Pos_Z = req.Pos.z;
        Plane_Angle_deg = req.Plane_Angle;

        Landing_Surface_Pose::Update_Pose();

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Landing_Surface_Pose);
}