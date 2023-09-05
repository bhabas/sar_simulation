#include <Motor_Plugin.h>


namespace gazebo
{

    void Motor_Plugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Motor_Plugin\n";
        Model_Ptr = parent;
        World_Ptr = Model_Ptr->GetWorld();


        // LOAD SAR_TYPE,SAR_CONFIG,AND CAM_CONFIG NAMES
        ros::param::get("/SAR_SETTINGS/SAR_Type",SAR_Type);
        ros::param::get("/SAR_SETTINGS/SAR_Config",SAR_Config);
        ros::param::get("/CAM_SETTINGS/Cam_Config",Cam_Config);

        // COLLECT MOTOR PARAMS FROM ROS
        ros::param::get("/SAR_Type/"+SAR_Type+"/System_Params/Thrust_Coeff",Thrust_Coeff);
        ros::param::get("/SAR_Type/"+SAR_Type+"/System_Params/Torque_Coeff",Torque_Coeff);
        ros::param::get("/SAR_Type/"+SAR_Type+"/System_Params/C_tf",C_tf);
        ros::param::get("/SAR_Type/"+SAR_Type+"/System_Params/Tau_up",Tau_up);
        ros::param::get("/SAR_Type/"+SAR_Type+"/System_Params/Tau_down",Tau_down);



        // GRAB MOTOR JOINT FROM SDF
        Motor_Joint_Name = _sdf->GetElement("Joint_Name")->Get<std::string>();
        gzmsg << "\t Joint Name:\t" << Motor_Joint_Name << std::endl;
        Joint_Ptr = Model_Ptr->GetJoint(Motor_Joint_Name);
        if (Joint_Ptr == NULL)
            gzerr << "[gazebo_motor_model] Couldn't find specified joint \"" << Motor_Joint_Name << std::endl;

        // GRAB ROTOR LINK FROM SDF
        Prop_Link_Name = _sdf->GetElement("Link_Name")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << Prop_Link_Name << std::endl;
        Link_Ptr = Model_Ptr->GetLink(Prop_Link_Name);
        if (Link_Ptr == NULL)
            gzerr << "[gazebo_motor_model] Couldn't find specified link \"" << Prop_Link_Name << std::endl;

        
        
        // COLLECT OTHER PARAMS FROM SDF
        Motor_Number = _sdf->GetElement("Motor_Number")->Get<int>();
        Rot_Vel_Slowdown = _sdf->GetElement("Visual_Slowdown")->Get<double>();
        

        // COLLECT MOTOR TURNING DIRECTION
        if (_sdf->GetElement("Turning_Direction")->Get<std::string>() == "ccw")
        {
            Turn_Direction = 1;
        }
        else if(_sdf->GetElement("Turning_Direction")->Get<std::string>() == "cw")
        {
            Turn_Direction = -1;
        }
        else
        {
            gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as Turning_Direction" << std::endl;
        }

        Prev_Sim_time = World_Ptr->SimTime().Double();

        // // // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Motor_Plugin::OnUpdate, this));

        // std::cout << "\n\n";
    }

    void Motor_Plugin::OnUpdate()
    {
        Sampling_time = World_Ptr->SimTime().Double() - Prev_Sim_time;
        Prev_Sim_time = World_Ptr->SimTime().Double();
        UpdateForcesAndMoments();

        // SET VISUAL VELOCTIY OF ROTOR
        Rot_Vel = sqrt(Thrust/Thrust_Coeff);
        Joint_Ptr->SetVelocity(0,Turn_Direction * Rot_Vel / Rot_Vel_Slowdown);
    }

    void Motor_Plugin::UpdateForcesAndMoments()
    {
        // UPDATE THRUST VIA FIRST ORDER FILTER (INSTANTANEOUS THRUSTS ARE NOT POSSIBLE)
        if (Thrust_input >= Prev_Thrust)
        {
            // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
            double alpha_up = exp(-Sampling_time/Tau_up);
            Thrust = alpha_up*Prev_Thrust + (1-alpha_up)*Thrust_input;
        }
        else // Thrust_input < prev_thrust
        {
            // x(k) = alpha*x(k-1) + (1-alpha)*u(k)
            double alpha_down = exp(-Sampling_time/Tau_down);
            Thrust = alpha_down*Prev_Thrust + (1-alpha_down)*Thrust_input;
        }
        
        
        // APPLY ROTOR THRUST TO LINK
        Link_Ptr->AddRelativeForce(ignition::math::Vector3d(0, 0, (Thrust*g2Newton)));

        // APPLY ROTOR TORQUE TO MAIN BODY
        Torque = Torque_Coeff*(Thrust*g2Newton);
        ignition::math::Vector3d Torque_Vec(0, 0, -Turn_Direction * Torque); // Torque is opposite direction of rotation

        physics::Link_V Parent_Links = Link_Ptr->GetParentJointsLinks(); // Get <vector> of parent links
        ignition::math::Pose3d Pose_Difference = Link_Ptr->WorldCoGPose() - Parent_Links.at(0)->WorldCoGPose(); // Find rotor pos relative to body
        ignition::math::Vector3d Torque_Parent_Frame = Pose_Difference.Rot().RotateVector(Torque_Vec); // Rotate Torque vector to match body orientation
        Parent_Links.at(0)->AddRelativeTorque(Torque_Parent_Frame); // Apply Torque vector to body
    

        Prev_Thrust = Thrust;

        Thrust_msg.Motor_Number = Motor_Number;
        Thrust_msg.MotorThrust = Thrust_input;
        Thrust_msg.MotorThrust_actual = Thrust;
        MS_Data_Pub.publish(Thrust_msg);

    }


    void Motor_Plugin::CtrlData_Callback(const sar_msgs::CTRL_Data::ConstPtr &msg)
    {
        Thrust_input = msg->MotorThrusts[Motor_Number-1];
    }

    GZ_REGISTER_MODEL_PLUGIN(Motor_Plugin);
}