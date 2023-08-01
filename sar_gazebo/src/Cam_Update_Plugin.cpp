
#include <Cam_Update_Plugin.h>



namespace gazebo
{

    void Cam_Update_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Camera_Update_Plugin\n";

        // LOAD MODEL AND LINK POINTERS
        Base_Model_Ptr = _parent;
        SAR_Body_Ptr = _parent->GetLink("SAR_Body");
        Camera_Link_Ptr = _parent->GetLink("Camera");

        // LOAD PARAMS FROM SDF
        Joint_Name = _sdf->GetElement("jointName")->Get<std::string>();


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
        Sensor_Ptr = sensors::get_sensor("Camera");
        Camera_Ptr = dynamic_cast<sensors::CameraSensor*>(Sensor_Ptr.get());

        // UPDATE CAMERA
        Cam_Update_Plugin::Update_Camera();

        
        // LINK COMMAND SERVICE TO CALLBACK
        // Pose_Update_Service = nh.advertiseService("/Cam_Update_Plugin", &Cam_Update_Plugin::Service_Callback, this);

        printf("\n\n");
    }

    void Cam_Update_Plugin::Update_Camera()
    {
        // UPDATE CAMERA POSE
        gzmsg << "Removing Camera-to-Model Joint\n";
        Base_Model_Ptr->RemoveJoint(Joint_Name);

        gzmsg << "Updating Relative Pose\n";
        ignition::math::Pose3d relativePose(X_Offset, Y_Offset, Z_Offset, 0.0, (90 - Pitch_Angle)*M_PI/180.0, 0.0);
        Camera_Link_Ptr->SetRelativePose(relativePose);

        gzmsg << "Creating Camera-to-Model Joint\n";
        Base_Model_Ptr->CreateJoint(Joint_Name,"fixed",SAR_Body_Ptr,Camera_Link_Ptr);

        // UPDATE CAMERA SETTINGS
        Camera_Ptr->SetUpdateRate(FPS);

    }

    bool Cam_Update_Plugin::Service_Callback(gazebo_msgs::SetModelState::Request &req, gazebo_msgs::SetModelState::Response &res)
    {
        Cam_Update_Plugin::Update_Camera();

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Cam_Update_Plugin);
}