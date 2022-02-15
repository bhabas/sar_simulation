#include <iostream>
#include <gazebo_sticky_foot.h>

/*
    This plugin is responsible for joining foot pads to whatever entity they collide with (e.g. ground or ceiling).
*/

namespace gazebo{


    // This gets called when model is loaded and pulls values from sdf file
    void GazeboStickyFoot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
        model_ =_model;

        gzmsg << "Loading GazeboStickyFoot Plugin\n";
        std::cout << "Loading GazeboStickyFoot Plugin2\n" << std::endl;
    }


GZ_REGISTER_MODEL_PLUGIN(GazeboStickyFoot);
}