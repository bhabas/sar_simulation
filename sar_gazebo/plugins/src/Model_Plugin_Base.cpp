#include <Model_Plugin_Base.h>


namespace gazebo
{
    void Model_Plugin_Base::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading *******_Plugin\n";


        printf("\n\n");
    }


    GZ_REGISTER_MODEL_PLUGIN(Model_Plugin_Base);
}