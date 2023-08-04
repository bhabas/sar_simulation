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
        Base_Model_Ptr = _parent;

        SAR_Body_Ptr = Base_Model_Ptr->GetLink("Base_Model::SAR_Body");
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

        std::cout << Inertial_Ptr->Mass() << std::endl;
        std::cout << Inertial_Ptr->IXX() << std::endl;
        std::cout << Inertial_Ptr->IYY() << std::endl;
        std::cout << Inertial_Ptr->IZZ() << std::endl;

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
        
        std::cout << Inertial_Ptr->Mass() << std::endl;
        std::cout << Inertial_Ptr->IXX() << std::endl;
        std::cout << Inertial_Ptr->IYY() << std::endl;
        std::cout << Inertial_Ptr->IZZ() << std::endl;



        // Camera_Link_Ptr = Base_Model_Ptr->GetLink("Base_Model::Camera");
        // if (!Camera_Link_Ptr)
        // {
        //     gzerr << "Camera link is NULL.\n";
        //     return;
        // }

        // // // LOAD PARAMS FROM SDF
        // // Joint_Name = _sdf->GetElement("jointName")->Get<std::string>();

        // // // LOAD SAR_Type,SAR_Config,AND CAM_CONFIG PARAMS
        // // ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
        // // ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);
        // // ros::param::get("/CAM_SETTINGS/Cam_Config",Cam_Config);

        // // // LOAD CAMERA CONFIG SETTINGS
        // // ros::param::get("/Cam_Config/"+Cam_Config+"/X_Offset",X_Offset);
        // // ros::param::get("/Cam_Config/"+Cam_Config+"/Y_Offset",Y_Offset);
        // // ros::param::get("/Cam_Config/"+Cam_Config+"/Z_Offset",Z_Offset);
        // // ros::param::get("/Cam_Config/"+Cam_Config+"/Pitch_Angle",Pitch_Angle);
        // // ros::param::get("/Cam_Config/"+Cam_Config+"/FPS",FPS);

        // // // GET POINTER TO CAMERA SENSOR
        // // Sensor_Ptr = sensors::get_sensor("Base_Model::Camera");
        // // Camera_Ptr = dynamic_cast<sensors::CameraSensor*>(Sensor_Ptr.get());




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