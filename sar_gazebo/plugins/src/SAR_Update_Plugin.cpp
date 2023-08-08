#include <SAR_Update_Plugin.h>

// UPDATE INTERTIAL VALUES
//    USE THIS FOR DOMAIN RANDOMIZATION
// UPDATE LEG STIFFNESS AND DAMPING (----)
// MOTOR THRUST COEFFICIENTS
// UPDATE SIM SPEED BASED ON SPEED SETTING ON STARTUP (----)

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

        World_Ptr = Config_Model_Ptr->GetWorld();
        
        
        double Sim_Speed;
        double Step_Size = 0.001;
        ros::param::get("/SIM_SETTINGS/Sim_Speed",Sim_Speed);
        World_Ptr->Physics()->SetRealTimeUpdateRate(Sim_Speed/Step_Size);



        double Ixx_Body;
        double Iyy_Body;
        double Izz_Body;
        double Mass_Body;

        // LOAD SAR_Type,SAR_Config,AND CAM_CONFIG PARAMS
        ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
        ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);
        ros::param::get("/CAM_SETTINGS/Cam_Config",Cam_Config);

        // LOAD BASE_MODEL INTERTIAL PARAMS
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Ixx",Ixx_Body);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Iyy",Iyy_Body);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Izz",Izz_Body);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/Base_Model/Mass",Mass_Body);


        // UPDATE BASE LINK INERTIA
        Inertial_Ptr->SetIXX(Ixx_Body);
        Inertial_Ptr->SetIYY(Iyy_Body);
        Inertial_Ptr->SetIZZ(Izz_Body);
        Inertial_Ptr->SetMass(Mass_Body);


        // GET LEG LINK POINTERS
        Leg_1_LinkPtr = Config_Model_Ptr->GetLink("Leg_1");
        Leg_2_LinkPtr = Config_Model_Ptr->GetLink("Leg_2");
        Leg_3_LinkPtr = Config_Model_Ptr->GetLink("Leg_3");
        Leg_4_LinkPtr = Config_Model_Ptr->GetLink("Leg_4");

        // GET LEG HINGE POINTERS
        Hinge_1_JointPtr = Config_Model_Ptr->GetJoint("Hinge_1_Joint");
        Hinge_2_JointPtr = Config_Model_Ptr->GetJoint("Hinge_2_Joint");
        Hinge_3_JointPtr = Config_Model_Ptr->GetJoint("Hinge_3_Joint");
        Hinge_4_JointPtr = Config_Model_Ptr->GetJoint("Hinge_4_Joint");

        

        Update_Hinge_Params();

        

        printf("\n\n");
    }

    void SAR_Update_Plugin::Update_Camera()
    {
        gzmsg << "Updating Camera Pose and FPS\n";

    }

    void SAR_Update_Plugin::Update_Hinge_Params()
    {
        gzmsg << "Updating Leg Hinge Joint Params\n";

        // GET LEG INERTIAL VALUES
        Iyy_Leg = Leg_1_LinkPtr->GetInertial()->IYY();
        Ixx_Leg = Leg_1_LinkPtr->GetInertial()->IXX();
        Izz_Leg = Leg_1_LinkPtr->GetInertial()->IZZ();

        // LOAD LEG PARAMETERS
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/Leg_Angle",Leg_Angle);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/K_Pitch",K_Pitch);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/DR_Pitch",DR_Pitch);

        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/K_Yaw",K_Yaw);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/DR_Yaw",DR_Yaw);

        // CALC DAMPING COEFFICIENTS
        C_Pitch = DR_Pitch*2*sqrt(K_Pitch*Iyy_Leg);
        C_Yaw = DR_Yaw*2*sqrt(K_Yaw*Izz_Leg);

        // UPDATE JOINTS
        Hinge_1_JointPtr->SetUpperLimit(Y_AXIS,(20 + Leg_Angle)*Deg2Rad);
        Hinge_1_JointPtr->SetLowerLimit(Y_AXIS,-(60 - Leg_Angle)*Deg2Rad);
        Hinge_1_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_1_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);

        Hinge_2_JointPtr->SetUpperLimit(Y_AXIS,(20 + Leg_Angle)*Deg2Rad);
        Hinge_2_JointPtr->SetLowerLimit(Y_AXIS,-(60 - Leg_Angle)*Deg2Rad);
        Hinge_2_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_2_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);

        Hinge_3_JointPtr->SetUpperLimit(Y_AXIS,(20 + Leg_Angle)*Deg2Rad);
        Hinge_3_JointPtr->SetLowerLimit(Y_AXIS,-(60 - Leg_Angle)*Deg2Rad);
        Hinge_3_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_3_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);

        Hinge_4_JointPtr->SetUpperLimit(Y_AXIS,(20 + Leg_Angle)*Deg2Rad);
        Hinge_4_JointPtr->SetLowerLimit(Y_AXIS,-(60 - Leg_Angle)*Deg2Rad);
        Hinge_4_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_4_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);

    }

    bool SAR_Update_Plugin::Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res)
    {

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(SAR_Update_Plugin);
}