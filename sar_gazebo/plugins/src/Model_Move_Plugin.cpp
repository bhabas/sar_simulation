#include <iostream>
#include <Model_Move_Plugin.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void Model_Move::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMomentPlugin\n";
        model_ptr = parent;

        // LINK COMMAND SERVICE TO CALLBACK
        CMD_Service = nh.advertiseService("/Model_Movement", &Model_Move::Service_Callback, this);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Model_Move::OnUpdate, this));
        printf("\n\n");

    }

    void Model_Move::OnUpdate()
    {
        // CALC TIME SINCE START
        t_delta = ros::Time::now().toSec() - t_0;

        
        // CALC ANGULAR VELOCITY TO MATCH DESIRED CONDITIONS
        // (Updating velocity prevents collision box intersections that position updates cause through teleportation)
        Ang_Vel.X() = Eul_Lim.X() * 2*M_PI*Freq.X() * cos(2*M_PI*Freq.X()*t_delta);
        Ang_Vel.Y() = Eul_Lim.Y() * 2*M_PI*Freq.Y() * cos(2*M_PI*Freq.Y()*t_delta);
        Ang_Vel.Z() = Eul_Lim.Z() * 2*M_PI*Freq.Z() * cos(2*M_PI*Freq.Z()*t_delta);
        
        // CALC VELOCITY TO MATCH DESIRED CONDITIONS
        Vel.X() = Vel_0.X() + Accel_0.X()*t_delta;
        Vel.Y() = Vel_0.Y() + Accel_0.Y()*t_delta;
        Vel.Z() = Vel_0.Z() + Accel_0.Z()*t_delta;

        // UPDATE MODEL VELOCITY AND ANGULAR VELOCITY
        model_ptr->SetWorldTwist(Vel,Ang_Vel);

    }


    bool Model_Move::Service_Callback(sar_msgs::Model_Move::Request &req, sar_msgs::Model_Move::Response &res)
    {
        // RESET TIME
        t_0 = ros::Time::now().toSec();


        // UPDATE STARTING POSITION AND ORIENTATION
        Pos_0.Set(req.Pos_0.x,req.Pos_0.y,req.Pos_0.z);
        Eul_0.Set(req.Eul_0.x,req.Eul_0.y,req.Eul_0.z);
        Eul_0 *= M_PI/180; // Convert to radians

        Pose_0.Set(Pos_0,Eul_0);
        model_ptr->SetWorldPose(Pose_0);
        

        // UPDATE VELOCITY PARAMETERS
        Vel_0.Set(req.Vel_0.x,req.Vel_0.y,req.Vel_0.z);
        Accel_0.Set(req.Accel_0.x,req.Accel_0.y,req.Accel_0.z);


        // UPDATE OSCILLATION PARAMETERS
        Freq.Set(req.Freq.x,req.Freq.y,req.Freq.z);
        Eul_Lim.Set(req.Eul_Lim.x,req.Eul_Lim.y,req.Eul_Lim.z);
        Eul_Lim *= M_PI/180; // Convert to radians
        

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Model_Move);
}