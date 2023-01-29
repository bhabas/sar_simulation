#include <iostream>
#include <Model_Move.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void ModelMove::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMomentPlugin\n";
        model_ptr = parent;

        linkName = _sdf->GetElement("linkName")->Get<std::string>();
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

        t_0 = ros::Time::now().toSec();
        vel.Set(req.Vel.x,req.Vel.y,req.Vel.z);
        ang_vel.Set(req.Ang_Vel.x,req.Ang_Vel.y,req.Ang_Vel.z);
        model_ptr->SetWorldTwist(vel,ang_vel);


        if (req.Reset_Pose == true)
        {
            pos.Set(req.Pos.x,req.Pos.y,req.Pos.z);
            eul.Set(req.Eul.x,req.Eul.y,req.Eul.z);
            pose.Set(pos,eul);
            model_ptr->SetWorldPose(pose);
        }
        
        

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(ModelMove);
}