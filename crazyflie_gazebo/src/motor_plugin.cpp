#include <iostream>
#include <motor_plugin.h>


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
        torque_coeff = _sdf->GetElement("rotorTorqueCoeff")->Get<double>();
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

        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMotorPlugin::OnUpdate, this));

        std::cout << "\n\n";
    }

    void GazeboMotorPlugin::OnUpdate()
    {
        updateThrust();
        updateTorque();
        
        // SET VISUAL VELOCTIY OF ROTOR
        rot_vel = sqrt(thrust/thrust_coeff);
        joint_ptr->SetVelocity(0,turning_direction * rot_vel / rot_vel_visual_slowdown);
    }

    void GazeboMotorPlugin::updateThrust()
    {
        // APPLY ROTOR THRUST TO LINK
        thrust = MotorThrust*g2Newton;
        link_ptr->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

    }

    void GazeboMotorPlugin::updateTorque()
    {
        // APPLY ROTOR TORQUE TO MAIN BODY
        torque = torque_coeff*thrust;
        ignition::math::Vector3d torque_vec(0, 0, -turning_direction * torque); // Torque is opposite direction of rotation

        physics::Link_V parent_links = link_ptr->GetParentJointsLinks(); // Get <vector> of parent links
        ignition::math::Pose3d pose_difference = link_ptr->WorldCoGPose() - parent_links.at(0)->WorldCoGPose(); // Find rotor pos relative to body
        ignition::math::Vector3d torque_parent_frame = pose_difference.Rot().RotateVector(torque_vec); // Rotate torque vector to match body orientation
        parent_links.at(0)->AddRelativeTorque(torque_parent_frame); // Apply torque vector to body
    }

    void GazeboMotorPlugin::CtrlData_Callback(const crazyflie_msgs::CtrlData::ConstPtr &msg)
    {
        MotorThrust = msg->MotorThrusts[motor_number-1];
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);
}