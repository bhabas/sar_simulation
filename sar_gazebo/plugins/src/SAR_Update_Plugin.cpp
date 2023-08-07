#include <SAR_Update_Plugin.h>

// UPDATE INTERTIAL VALUES
//    USE THIS FOR DOMAIN RANDOMIZATION
// UPDATE COG LOCATION
// UPDATE LEG STIFFNESS AND DAMPING
// MOTOR THRUST COEFFICIENTS
// UPDATE SIM SPEED BASED ON SPEED SETTING ON STARTUP

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
       
        physics::LinkPtr Leg_1_LinkPtr = Config_Model_Ptr->GetLink("Leg_1");
        physics::LinkPtr Leg_2_LinkPtr = Config_Model_Ptr->GetLink("Leg_2");
        physics::LinkPtr Leg_3_LinkPtr = Config_Model_Ptr->GetLink("Leg_3");
        physics::LinkPtr Leg_4_LinkPtr = Config_Model_Ptr->GetLink("Leg_4");

        
        physics::JointPtr Hinge_1_JointPtr = Config_Model_Ptr->GetJoint("Hinge_1_Joint");
        physics::JointPtr Hinge_2_JointPtr = Config_Model_Ptr->GetJoint("Hinge_2_Joint");
        physics::JointPtr Hinge_3_JointPtr = Config_Model_Ptr->GetJoint("Hinge_3_Joint");
        physics::JointPtr Hinge_4_JointPtr = Config_Model_Ptr->GetJoint("Hinge_4_Joint");

        double Iyy_Leg = Leg_1_LinkPtr->GetInertial()->IYY();
        double Ixx_Leg = Leg_1_LinkPtr->GetInertial()->IXX();
        double Izz_Leg = Leg_1_LinkPtr->GetInertial()->IZZ();

        double K_Pitch;
        double DR_Pitch;
        double C_Pitch;

        double K_Yaw;
        double DR_Yaw;
        double C_Yaw;

        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/K_Pitch",K_Pitch);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/DR_Pitch",DR_Pitch);

        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/K_Yaw",K_Yaw);
        ros::param::get("/SAR_Type/"+SAR_Type+"/Config/"+SAR_Config+"/DR_Yaw",DR_Yaw);

        C_Pitch = DR_Pitch*2*sqrt(K_Pitch*Iyy_Leg);
        C_Yaw = DR_Yaw*2*sqrt(K_Yaw*Izz_Leg);


        // Leg_1_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_1_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Hinge_1_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_1_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);



        // Leg_2_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_2_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Hinge_2_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_2_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);



        // Leg_3_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_3_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Hinge_3_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_3_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);


        // Leg_4_JointPtr->SetUpperLimit(Y_AXIS,5*Deg2Rad);
        // Leg_4_JointPtr->SetLowerLimit(Y_AXIS,-80*Deg2Rad);
        Hinge_4_JointPtr->SetStiffnessDamping(Y_AXIS,K_Pitch,C_Pitch,0);
        Hinge_4_JointPtr->SetStiffnessDamping(Z_AXIS,K_Yaw,C_Yaw,0);


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