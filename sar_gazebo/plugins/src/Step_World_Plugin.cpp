#include <Step_World_Plugin.h>


namespace gazebo
{
    void Step_World_Plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Step_World_Plugin\n";


        printf("\n\n");
    }


    GZ_REGISTER_WORLD_PLUGIN(Step_World_Plugin);
}