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
        t_delta = ros::Time::now().toSec() - t_0;

        double val = Eul_Lim.Y() * 2*M_PI*Freq.Y() * cos(2*M_PI*Freq.Y()*t_delta);
        model_ptr->SetWorldTwist(Vel,ignition::math::Vector3d(0, val, 0));

    }


    bool ModelMove::Service_Callback(crazyflie_msgs::ModelMove::Request &req, crazyflie_msgs::ModelMove::Response &res)
    {

        t_0 = ros::Time::now().toSec();
        Vel.Set(req.Vel.x,req.Vel.y,req.Vel.z);

        Eul_Lim.Set(req.Eul_Lim.x,req.Eul_Lim.y,req.Eul_Lim.z);
        Eul_Lim *= M_PI/180;

        Freq.Set(req.Freq.x,req.Freq.y,req.Freq.z);



        if (req.Reset_Pose == true)
        {
            Pos.Set(req.Pos.x,req.Pos.y,req.Pos.z);
            Eul_0.Set(req.Eul_0.x,req.Eul_0.y,req.Eul_0.z);
            Eul_0 *= M_PI/180;

            Pose.Set(Pos,Eul_0);
            model_ptr->SetWorldPose(Pose);
        }
        
        

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(ModelMove);
}