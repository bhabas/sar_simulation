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
        gzmsg << "Loading GazeboMotorPlugin\n";
        model_ = parent;

        // GRAB ROTOR JOINT FROM SDF
        jointName = _sdf->GetElement("jointName")->Get<std::string>();
        gzmsg << "\t Joint Name:\t" << jointName << std::endl;
        joint_ptr = model_->GetJoint(jointName);
        if (joint_ptr == NULL)
            gzerr << "[gazebo_motor_model] Couldn't find specified joint \"" << jointName << std::endl;

        // GRAB ROTOR LINK FROM SDF
        linkName = _sdf->GetElement("linkName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_->GetLink(linkName);
        if (link_ptr == NULL)
            gzerr << "[gazebo_motor_model] Couldn't find specified link \"" << linkName << std::endl;
        
        // COLLECT OTHER PARAMS FROM SDF
        motor_number = _sdf->GetElement("motorNumber")->Get<int>();
        thrust_coeff = _sdf->GetElement("rotorThrustCoeff")->Get<double>();
        thrust_coeff = _sdf->GetElement("rotorTorqueCoeff")->Get<double>();
        rot_vel_visual_slowdown = _sdf->GetElement("rotorVelocitySlowdownSim")->Get<double>();

        if (_sdf->GetElement("turningDirection")->Get<std::string>() == "ccw")
        {
            turning_direction = 1;
        }
        else if(_sdf->GetElement("turningDirection")->Get<std::string>() == "cw")
        {
            turning_direction = -1;
        }
        else
        {
            gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as turningDirection" << std::endl;
        }

 
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMotorPlugin::OnUpdate, this));

        std::cout << "\n\n";
    }

    void GazeboMotorPlugin::OnUpdate()
    {
        updateThrust();
        
        joint_ptr->SetVelocity(0,turning_direction * rot_vel / rot_vel_visual_slowdown);
    }

    void GazeboMotorPlugin::updateThrust()
    {
        // double thrust = calculateThrust(cmd.velocity[i]);
        // double torque = calculateTorque(cmd.velocity[i]);
    }

    void GazeboMotorPlugin::MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg)
    {
        rot_vel = msg->MotorPWM[motor_number-1];
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);
}