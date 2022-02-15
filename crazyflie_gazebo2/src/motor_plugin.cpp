#include <iostream>
#include <motor_plugin.h>

/*
    This plugin is responsible for joining foot pads to whatever entity they collide with (e.g. ground or ceiling).
*/

namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void GazeboMotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        
    }

    void GazeboMotorPlugin::MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg)
    {

    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);
}