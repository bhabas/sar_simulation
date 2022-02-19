#include <iostream>
#include <motor_plugin.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void GazeboMotorPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMotorPlugin\n";
        model_ = parent;

        

        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMotorPlugin::OnUpdate, this));

        std::cout << "\n\n";
    }

    void GazeboMotorPlugin::OnUpdate()
    {
        
    }



    // void GazeboMotorPlugin::MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg)
    // {

    // }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);
}