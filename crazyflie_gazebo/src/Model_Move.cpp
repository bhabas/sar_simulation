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

        CMD_Service = nh.advertiseService("/ModelMovement", &ModelMove::Service_Callback, this);


        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelMove::OnUpdate, this));
        printf("\n\n");

    }

    void ModelMove::OnUpdate()
    {
        
        // model_ptr->SetLinearVel(vel_vec);
    }


    bool ModelMove::Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res)
    {
        // vel_vec.Set(req.Vel.x,req.Vel.y,req.Vel.z);
        t_0 = ros::Time::now().toSec();
        

        if (req.Reset_Pose == true)
        {
            pose.Set(req.Position.x,0,0,0,0,0);
            model_ptr->SetWorldPose(pose);
        }
        
        

        return true;
    }

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

    GZ_REGISTER_MODEL_PLUGIN(ModelMove);
}