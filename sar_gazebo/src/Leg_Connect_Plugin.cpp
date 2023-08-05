
#include <Leg_Connect_Plugin.h>

namespace gazebo
{

    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";

      

        printf("\n\n");
    }

    void Leg_Connect_Plugin::Update_Camera()
    {
        gzmsg << "Updating Camera Pose and FPS\n";

    }

    bool Leg_Connect_Plugin::Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res)
    {

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}