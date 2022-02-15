#include <iostream>
#include <motor_plugin.h>

/*
    This plugin is responsible for joining foot pads to whatever entity they collide with (e.g. ground or ceiling).
*/

namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void GazeboMotorPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Motor Plugin\n";
        _model = parent;


        // LOAD SDF VALUES
        if (_sdf->HasElement("motorNumber")) 
            motor_number = _sdf->GetElement("motorNumber")->Get<int>();
        else 
            gzerr << "[gazebo_motor_model] Please specify a motorNumber.\n";

        if (_sdf->HasElement("rotorThrustCoeff")) 
            thrust_coeff = _sdf->GetElement("rotorThrustCoeff")->Get<double>();
        else 
            gzerr << "[gazebo_motor_model] Please specify a Rotor Thrust Coefficient.\n";

        if (_sdf->HasElement("rotorTorqueCoeff")) 
            thrust_coeff = _sdf->GetElement("rotorTorqueCoeff")->Get<double>();
        else 
            gzerr << "[gazebo_motor_model] Please specify a Rotor Torque Coefficient.\n";

        if (_sdf->HasElement("turningDirection")) {
            std::string turning_direction = _sdf->GetElement("turningDirection")->Get<std::string>();
            if (turning_direction == "cw")
                turning_direction = 1;
            else if (turning_direction == "ccw")
                turning_direction = -1;
            else
                gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection.\n";
        }
        else
            gzerr << "[gazebo_motor_model] Please specify a turning direction ('cw' or 'ccw').\n";


        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMotorPlugin::OnUpdate, this));

    }

    void GazeboMotorPlugin::OnUpdate()
    {
        tick += 1;
        std::cout << "Hello World:\t" << tick << std::endl;
    }

    void GazeboMotorPlugin::MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg)
    {

    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);
}