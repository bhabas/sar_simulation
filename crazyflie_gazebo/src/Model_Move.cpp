#include <iostream>
#include <Model_Move.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void ModelMove::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMomentPlugin\n";
        model_ptr = parent;

        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_ptr->GetLink(linkName);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelMove::OnUpdate, this));
        printf("\n\n");

    }

    void ModelMove::OnUpdate()
    {
        vel_vec.Set(0.5,0,0);
        link_ptr->SetLinearVel(vel_vec);
    }

    // void ModelMove::RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
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

    GZ_REGISTER_MODEL_PLUGIN(ModelMove);
}