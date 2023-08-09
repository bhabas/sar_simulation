#include <iostream>
#include <Motor_Plugin.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void GazeboMotorPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMotorPlugin\n";
        model_ = parent;
        world_ = model_->GetWorld();



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
        rot_vel_visual_slowdown = _sdf->GetElement("rotorVisualSlowdown")->Get<double>();
        timeConstantUp = _sdf->GetElement("timeConstantUp")->Get<double>();
        timeConstantDown = _sdf->GetElement("timeConstantDown")->Get<double>();

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

        prev_sim_time = world_->SimTime().Double();

        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboMotorPlugin::OnUpdate, this));

        std::cout << "\n\n";
    }

    void GazeboMotorPlugin::OnUpdate()
    {
        sampling_time = world_->SimTime().Double() - prev_sim_time;
        prev_sim_time = world_->SimTime().Double();

        UpdateForcesAndMoments();

   
        
        // SET VISUAL VELOCTIY OF ROTOR
        rot_vel = sqrt(thrust/thrust_coeff);
        joint_ptr->SetVelocity(0,turning_direction * rot_vel / rot_vel_visual_slowdown);
    }

    void GazeboMotorPlugin::UpdateForcesAndMoments()
    {
        // UPDATE THRUST VIA FIRST ORDER FILTER (INSTANTANEOUS THRUSTS ARE NOT POSSIBLE)
        if (MotorThrust_input >= prev_thrust)
        {
            // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
            double alpha_up = exp(-sampling_time/timeConstantUp);
            thrust = alpha_up*prev_thrust + (1-alpha_up)*MotorThrust_input;
        }
        else // MotorThrust_input < prev_thrust
        {
            double alpha_down = exp(-sampling_time/timeConstantDown);
            thrust = alpha_down*prev_thrust + (1-alpha_down)*MotorThrust_input;
        }
        
        
        // APPLY ROTOR THRUST TO LINK
        link_ptr->AddRelativeForce(ignition::math::Vector3d(0, 0, (thrust*g2Newton)));

        // APPLY ROTOR TORQUE TO MAIN BODY
        torque = torque_coeff*(thrust*g2Newton);
        ignition::math::Vector3d torque_vec(0, 0, -turning_direction * torque); // Torque is opposite direction of rotation

        physics::Link_V parent_links = link_ptr->GetParentJointsLinks(); // Get <vector> of parent links
        ignition::math::Pose3d pose_difference = link_ptr->WorldCoGPose() - parent_links.at(0)->WorldCoGPose(); // Find rotor pos relative to body
        ignition::math::Vector3d torque_parent_frame = pose_difference.Rot().RotateVector(torque_vec); // Rotate torque vector to match body orientation
        parent_links.at(0)->AddRelativeTorque(torque_parent_frame); // Apply torque vector to body
    

        prev_thrust = thrust;

        Thrust_msg.Motor_Number = motor_number;
        Thrust_msg.MotorThrust = MotorThrust_input;
        Thrust_msg.MotorThrust_actual = thrust;
        MS_Data_Pub.publish(Thrust_msg);

    }


    void GazeboMotorPlugin::CtrlData_Callback(const sar_msgs::CTRL_Data::ConstPtr &msg)
    {
        MotorThrust_input = msg->MotorThrusts[motor_number-1];
        
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorPlugin);
}