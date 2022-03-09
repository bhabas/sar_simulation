#include <iostream>
#include <model_Moment.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void ModelMoment::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMomentPlugin\n";
        model_ptr = parent;

        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_ptr->GetLink(linkName);
        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelMoment::OnUpdate, this));
        printf("\n\n");

    }

    void ModelMoment::OnUpdate()
    {
        // printf("flag: %u\n",executeMoment);
        if(executeMoment == true)
        {
            link_ptr->AddRelativeTorque(torque_vec);
            std::cout << model_ptr->RelativeAngularAccel() << std::endl;

        }
    }

    // void GazeboMotorPlugin::updateTorque()
    // {
    //     // APPLY ROTOR TORQUE TO MAIN BODY
    //     torque = torque_coeff*thrust;
    //     ignition::math::Vector3d torque_vec(0, 0, -turning_direction * torque); // Torque is opposite direction of rotation

    //     physics::Link_V parent_links = link_ptr->GetParentJointsLinks(); // Get <vector> of parent links
    //     ignition::math::Pose3d pose_difference = link_ptr->WorldCoGPose() - parent_links.at(0)->WorldCoGPose(); // Find rotor pos relative to body
    //     ignition::math::Vector3d torque_parent_frame = pose_difference.Rot().RotateVector(torque_vec); // Rotate torque vector to match body orientation
    //     parent_links.at(0)->AddRelativeTorque(torque_parent_frame); // Apply torque vector to body
    // }

    void ModelMoment::RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
    {
        if(msg->cmd_type == 50)
        {
            if(msg->cmd_flag == 1)
            {
                executeMoment = true;
                torque_vec.Set(msg->cmd_vals.x, msg->cmd_vals.y, msg->cmd_vals.z);
            }
            else
            {
                executeMoment = false;
            }
        }
        else if(msg->cmd_type == 0)
        {
            ignition::math::Pose3d pose(
                ignition::math::Vector3d(0, 0, 0.5),
                ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)
            );
            model_ptr->SetWorldPose(pose);
            model_ptr->SetWorldTwist(ignition::math::Vector3d(0, 0, 0),ignition::math::Vector3d(0, 0, 0));
            torque_vec.Set(0,0,0);

        }
    }

    GZ_REGISTER_MODEL_PLUGIN(ModelMoment);
}