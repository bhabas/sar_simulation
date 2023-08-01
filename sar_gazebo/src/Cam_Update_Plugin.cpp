
#include <Cam_Update_Plugin.h>

/**
 * @brief This script is responsible for updating the camera position, pitch angle, and FPS via ROS service
 * commands and adjusting these values from the Cam_Settings on simulation startup
 * 
 */

namespace gazebo
{

    void Cam_Update_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Camera_Update_Plugin\n";

        // LOAD MODEL AND LINK POINTERS
        Base_Model_Ptr = _parent;

        SAR_Body_Ptr = Base_Model_Ptr->GetLink("Base_Model::SAR_Body");
        if (!SAR_Body_Ptr)
        {
            gzerr << "SAR_Body link is NULL.\n";
            return;
        }

        Camera_Link_Ptr = Base_Model_Ptr->GetLink("Base_Model::Camera");
        if (!Camera_Link_Ptr)
        {
            gzerr << "Camera link is NULL.\n";
            return;
        }

        // LOAD PARAMS FROM SDF
        Joint_Name = _sdf->GetElement("jointName")->Get<std::string>();
        std::cout << Joint_Name << std::endl;

        // LOAD SAR_Type,SAR_Config,AND CAM_CONFIG PARAMS
        ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
        ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);
        ros::param::get("/CAM_SETTINGS/Cam_Config",Cam_Config);

        // LOAD CAMERA CONFIG SETTINGS
        ros::param::get("/Cam_Config/"+Cam_Config+"/X_Offset",X_Offset);
        ros::param::get("/Cam_Config/"+Cam_Config+"/Y_Offset",Y_Offset);
        ros::param::get("/Cam_Config/"+Cam_Config+"/Z_Offset",Z_Offset);
        ros::param::get("/Cam_Config/"+Cam_Config+"/Pitch_Angle",Pitch_Angle);
        ros::param::get("/Cam_Config/"+Cam_Config+"/FPS",FPS);

        // GET POINTER TO CAMERA SENSOR
        Sensor_Ptr = sensors::get_sensor("Base_Model::Camera");
        Camera_Ptr = dynamic_cast<sensors::CameraSensor*>(Sensor_Ptr.get());




        // UPDATE CAMERA
        Cam_Update_Plugin::Update_Camera();

        // LINK COMMAND SERVICE TO CALLBACK
        Cam_Update_Service = nh.advertiseService("/SAR_Internal/Camera_Update", &Cam_Update_Plugin::Service_Callback, this);

        printf("\n\n");
    }

    void Cam_Update_Plugin::Update_Camera()
    {
        gzmsg << "Updating Camera Pose and FPS\n";

        // UPDATE CAMERA POSE
        // printf("Removing joint\n");
        Base_Model_Ptr->RemoveJoint(Joint_Name);

        // printf("Moving Link\n");
        ignition::math::Pose3d relativePose(X_Offset, Y_Offset, Z_Offset, 0.0, (90 - Pitch_Angle)*M_PI/180.0, 0.0);
        Camera_Link_Ptr->SetRelativePose(relativePose);

        // printf("Creating joint\n");
        Base_Model_Ptr->CreateJoint(Joint_Name,"fixed",SAR_Body_Ptr,Camera_Link_Ptr);

        Camera_Ptr->SetUpdateRate(FPS);


    }

    bool Cam_Update_Plugin::Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res)
    {
        // UPDATING CAMERA POSE AND FPS
        X_Offset = req.Pos_Offset.x;
        Y_Offset = req.Pos_Offset.y;
        Z_Offset = req.Pos_Offset.z;
        Pitch_Angle = req.Pitch_Angle;
        FPS = req.FPS;

        Cam_Update_Plugin::Update_Camera();

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Cam_Update_Plugin);
}