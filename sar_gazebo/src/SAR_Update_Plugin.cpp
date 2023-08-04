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
        
        physics::LinkPtr Leg_1_LinkPtr = Config_Model_Ptr->GetLink("leg1");
        physics::LinkPtr Leg_2_LinkPtr = Config_Model_Ptr->GetLink("leg2");
        physics::LinkPtr Leg_3_LinkPtr = Config_Model_Ptr->GetLink("leg3");
        physics::LinkPtr Leg_4_LinkPtr = Config_Model_Ptr->GetLink("leg4");

        
        physics::JointPtr Leg_1_JointPtr = Config_Model_Ptr->GetJoint("leg_joint_1");
        physics::JointPtr Leg_2_JointPtr = Config_Model_Ptr->GetJoint("leg_joint_2");
        physics::JointPtr Leg_3_JointPtr = Config_Model_Ptr->GetJoint("leg_joint_3");
        physics::JointPtr Leg_4_JointPtr = Config_Model_Ptr->GetJoint("leg_joint_4");

        // gzdbg << Leg_1_JointPtr->GetStiffness(0) << "\n";
        // gzdbg << Leg_1_JointPtr->GetStiffness(1) << "\n";
        // gzdbg << Leg_1_JointPtr->GetDamping(0) << "\n";
        // gzdbg << Leg_1_JointPtr->GetDamping(1) << "\n";

        // gzdbg << Leg_2_JointPtr->UpperLimit(0) << "\n";
        // gzdbg << Leg_2_JointPtr->LowerLimit(0) << "\n";
        // gzdbg << Leg_2_JointPtr->GetStopDissipation(0) << "\n";
        // gzdbg << Leg_2_JointPtr->GetStopStiffness(0) << "\n";

        // gzdbg << Leg_3_JointPtr->GetStiffness(0) << "\n";
        // gzdbg << Leg_3_JointPtr->GetStiffness(1) << "\n";
        // gzdbg << Leg_3_JointPtr->GetDamping(0) << "\n";
        // gzdbg << Leg_3_JointPtr->GetDamping(1) << "\n";

        // gzdbg << Leg_4_JointPtr->GetStiffness(0) << "\n";
        // gzdbg << Leg_4_JointPtr->GetStiffness(1) << "\n";
        // gzdbg << Leg_4_JointPtr->GetDamping(0) << "\n";
        // gzdbg << Leg_4_JointPtr->GetDamping(1) << "\n";

        double Iyy_Leg = Leg_1_LinkPtr->GetInertial()->IYY();
        double Ixx_Leg = Leg_1_LinkPtr->GetInertial()->IXX();
        double Izz_Leg = Leg_1_LinkPtr->GetInertial()->IZZ();

        double K_pitch = Leg_1_JointPtr->GetStiffness(0);
        double Zeta_pitch = 1;
        double C_pitch = Zeta_pitch*sqrt(Iyy_Leg*K_pitch);
        C_pitch = 0.000111355*10; // This is now overdamped. 
        std::cout << C_pitch << std::endl;;


        // Leg_1_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_1_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Leg_1_JointPtr->SetStiffnessDamping(Y_AXIS,K_pitch,C_pitch,0);


        // Leg_2_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_2_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Leg_2_JointPtr->SetStiffnessDamping(Y_AXIS,K_pitch,C_pitch,0);


        // Leg_3_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_3_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Leg_3_JointPtr->SetStiffnessDamping(Y_AXIS,K_pitch,C_pitch,0);

        // Leg_4_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_4_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Leg_4_JointPtr->SetStiffnessDamping(Y_AXIS,K_pitch,C_pitch,0);

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