#include <Example_Cam_Plugin.h>


namespace gazebo
{
    void Example_Cam_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading *******_Plugin\n";


        printf("\n\n");
    }


    GZ_REGISTER_MODEL_PLUGIN(Example_Cam_Plugin);
}