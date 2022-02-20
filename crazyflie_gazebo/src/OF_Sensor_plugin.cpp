#include <iostream>
#include "OF_Sensor_plugin.h"


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void OF_SensorPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading OF_SensorPlugin\n";
        model_ = parent;

        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_->GetLink(linkName);

        OF_Publisher = nh.advertise<crazyflie_msgs::OF_SensorData>("CF_Internal/OF_Sensor",1);


        publisherThread = std::thread(&OF_SensorPlugin::Publish_OF_Data, this);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OF_SensorPlugin::OnUpdate, this));

        // ROS_WARN("loaded ModelPush Plugin with parent...%s",model_->GetName().c_str());

        std::cout << "\n\n";
    }
    void OF_SensorPlugin::Publish_OF_Data()
    {
        
        ros::Rate rate(100);
        while(ros::ok)
        {
            // PUBLISH OPTICAL FLOW VALUES
            OF_Data_msg.Tau = Tau;
            OF_Data_msg.RREV = RREV;
            OF_Data_msg.OFx = OFx;
            OF_Data_msg.OFy = OFy;
            OF_Data_msg.d_ceil = d_ceil;

            OF_Publisher.publish(OF_Data_msg);
            rate.sleep();
        }
    }

    void OF_SensorPlugin::OnUpdate()
    {

        model_->SetLinearVel(ignition::math::Vector3d(0.5, 0.5, 2.5));
        d_ceil = h_ceiling - link_ptr->WorldPose().Z();
        
        
        // DEFINE VELOCITIES RELATIVE TO BODY ORIENTATION
        Vx_rel = link_ptr->RelativeLinearVel().X();
        Vy_rel = link_ptr->RelativeLinearVel().Y();
        Vz_rel = link_ptr->RelativeLinearVel().Z();

        // CALCULATE OPTICAL FLOW VALUES
        Tau = d_ceil/Vz_rel;
        RREV = Vz_rel/d_ceil;
        OFx = -Vy_rel/d_ceil;
        OFy = -Vx_rel/d_ceil;


        // Publish_OF_Data();

    }



    // void GazeboMotorPlugin::MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg)
    // {

    // }

    GZ_REGISTER_MODEL_PLUGIN(OF_SensorPlugin);
}