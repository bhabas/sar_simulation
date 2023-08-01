#include <iostream>
#include <Model_Moment_Plugin.h>


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

        rotor1_ptr = model_ptr->GetLink("rotor_1");
        rotor4_ptr = model_ptr->GetLink("rotor_4");
        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelMoment::OnUpdate, this));
        printf("\n\n");

    }

    void ModelMoment::OnUpdate()
    {

        if(executeMoment == true)
        {
            link_ptr->AddRelativeTorque(torque_vec);
            // rotor1_ptr->AddRelativeForce(thrust_vec);
            // rotor4_ptr->AddRelativeForce(thrust_vec);

            std::cout << model_ptr->RelativeAngularAccel() << std::endl;

        }
    }

    // void ModelMoment::RLCmd_Callback(const sar_msgs::RLCmd::ConstPtr &msg)
    // {
    //     if(msg->cmd_type == 50)
    //     {
    //         if(msg->cmd_flag == 1)
    //         {
    //             executeMoment = true;
    //             torque_vec.Set(msg->cmd_vals.x, msg->cmd_vals.y, msg->cmd_vals.z);
    //             thrust_vec.Set(msg->cmd_vals.x, msg->cmd_vals.y, msg->cmd_vals.z);
                
    //         }
    //         else
    //         {
    //             executeMoment = false;
    //         }
    //     }
    //     else if(msg->cmd_type == 0)
    //     {
    //         ignition::math::Pose3d pose(
    //             ignition::math::Vector3d(0, 0, 0.5),
    //             ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)
    //         );
    //         model_ptr->SetWorldPose(pose);
    //         model_ptr->SetWorldTwist(ignition::math::Vector3d(0, 0, 0),ignition::math::Vector3d(0, 0, 0));
    //         torque_vec.Set(0,0,0);
    //         thrust_vec.Set(0,0,0);

    //     }
    // }

    GZ_REGISTER_MODEL_PLUGIN(ModelMoment);
}