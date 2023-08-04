#include <SAR_Update_Plugin.h>

// UPDATE INTERTIAL VALUES
//    USE THIS FOR DOMAIN RANDOMIZATION
// UPDATE COG LOCATION
// UPDATE LEG STIFFNESS AND DAMPING
// MOTOR THRUST COEFFICIENTS

namespace gazebo
{
    void SAR_Update_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading SAR_Update_Plugin\n";

        // LOAD MODEL AND LINK POINTERS
        Config_Model_Ptr = _parent;
        SAR_Body_Ptr = Config_Model_Ptr->GetLink("Base_Model::SAR_Body");
        if (!SAR_Body_Ptr)
        {
            gzerr << "SAR_Body link is NULL.\n";
            return;
        }

        // Create a new inertial object
        Inertial_Ptr = SAR_Body_Ptr->GetInertial();

        double Ixx;
        double Iyy;
        double Izz;
        double Mass;

        // LOAD SAR_Type,SAR_Config,AND CAM_CONFIG PARAMS
        ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
        ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);
        ros::param::get("/CAM_SETTINGS/Cam_Config",Cam_Config);

        // LOAD BASE_MODEL INTERTIAL PARAMS
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Ixx",Ixx);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Iyy",Iyy);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Izz",Izz);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Mass",Mass);

        Inertial_Ptr->SetIXX(Ixx);
        Inertial_Ptr->SetIYY(Iyy);
        Inertial_Ptr->SetIZZ(Izz);
        Inertial_Ptr->SetMass(Mass);


        gazebo::physics::Joint_V joints = Config_Model_Ptr->GetJoints();

        for (auto &joint : joints) {
            gzdbg << "Joint name: " << joint->GetName() << "\n";
        }
        
        physics::JointPtr Leg_Joint1_Ptr = Config_Model_Ptr->GetJoint("leg_joint_1");
        physics::JointPtr Leg_Joint2_Ptr = Config_Model_Ptr->GetJoint("leg_joint_2");
        physics::JointPtr Leg_Joint3_Ptr = Config_Model_Ptr->GetJoint("leg_joint_3");
        physics::JointPtr Leg_Joint4_Ptr = Config_Model_Ptr->GetJoint("leg_joint_4");

        gzdbg << Leg_Joint1_Ptr->GetStiffness(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetStiffness(1) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetDamping(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetDamping(1) << "\n";

        gzdbg << Leg_Joint1_Ptr->UpperLimit(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->LowerLimit(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetStopDissipation(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetStopStiffness(0) << "\n";


        gzdbg << Leg_Joint1_Ptr->GetStiffness(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetStiffness(1) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetDamping(0) << "\n";
        gzdbg << Leg_Joint1_Ptr->GetDamping(1) << "\n";

        Leg_Joint1_Ptr->SetUpperLimit(0,5*Deg2Rad);
        Leg_Joint1_Ptr->SetLowerLimit(0,-80*Deg2Rad);
        Leg_Joint1_Ptr->SetStiffnessDamping(0,0,0,0);


        Leg_Joint2_Ptr->SetUpperLimit(0,5*Deg2Rad);
        Leg_Joint2_Ptr->SetLowerLimit(0,-80*Deg2Rad);
        Leg_Joint2_Ptr->SetStiffnessDamping(0,0,0,0);


        Leg_Joint3_Ptr->SetUpperLimit(0,5*Deg2Rad);
        Leg_Joint3_Ptr->SetLowerLimit(0,-80*Deg2Rad);
        Leg_Joint3_Ptr->SetStiffnessDamping(0,0,0,0);


        Leg_Joint4_Ptr->SetUpperLimit(0,5*Deg2Rad);
        Leg_Joint4_Ptr->SetLowerLimit(0,-80*Deg2Rad);
        Leg_Joint4_Ptr->SetStiffnessDamping(0,0,0,0);

      



        
        // UPDATE CAMERA
        // SAR_Update_Plugin::Update_Camera();

        // LINK COMMAND SERVICE TO CALLBACK
        // Cam_Update_Service = nh.advertiseService("/SAR_Internal/Camera_Update", &SAR_Update_Plugin::Service_Callback, this);

        printf("\n\n");
    }

    void SAR_Update_Plugin::Update_Camera()
    {
        gzmsg << "Updating Camera Pose and FPS\n";

    }

    bool SAR_Update_Plugin::Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res)
    {

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(SAR_Update_Plugin);
}