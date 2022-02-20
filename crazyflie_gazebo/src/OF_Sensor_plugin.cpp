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

        topicName = _sdf->GetElement("topicName")->Get<std::string>();
        updateRate = _sdf->GetElement("updateRate")->Get<int>();

        Tau_gaussianNoise = _sdf->GetElement("Tau_gaussianNoise")->Get<double>();
        OFx_gaussianNoise = _sdf->GetElement("OFx_gaussianNoise")->Get<double>();
        OFy_gaussianNoise = _sdf->GetElement("OFy_gaussianNoise")->Get<double>();
        RREV_gaussianNoise = _sdf->GetElement("RREV_gaussianNoise")->Get<double>();


        ros::param::get("/CEILING_HEIGHT",_H_CEILING);


        OF_Publisher = nh.advertise<crazyflie_msgs::OF_SensorData>(topicName,1);
        publisherThread = std::thread(&OF_SensorPlugin::Publish_OF_Data, this);

        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OF_SensorPlugin::OnUpdate, this));

        gzmsg << "\t Loading Completed" << std::endl;
        std::cout << "\n\n";
    }
    void OF_SensorPlugin::Publish_OF_Data()
    {
        
        ros::Rate rate(updateRate);
        while(ros::ok)
        {
            // PUBLISH OPTICAL FLOW VALUES
            OF_Data_msg.Tau = Tau + GaussianKernel(0,Tau_gaussianNoise);
            OF_Data_msg.OFx = OFx + GaussianKernel(0,OFx_gaussianNoise);
            OF_Data_msg.OFy = OFy + GaussianKernel(0,OFy_gaussianNoise);
            OF_Data_msg.RREV = RREV + GaussianKernel(0,RREV_gaussianNoise);
            OF_Data_msg.d_ceil = d_ceil;

            OF_Publisher.publish(OF_Data_msg);
            rate.sleep();
        }
    }

    void OF_SensorPlugin::OnUpdate()
    {

        model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 2.5));
        d_ceil = _H_CEILING - link_ptr->WorldPose().Z();
        
        
        // DEFINE VELOCITIES RELATIVE TO BODY ORIENTATION
        Vx_rel = link_ptr->RelativeLinearVel().X();
        Vy_rel = link_ptr->RelativeLinearVel().Y();
        Vz_rel = link_ptr->RelativeLinearVel().Z();

        // CALCULATE OPTICAL FLOW VALUES
        Tau = d_ceil/Vz_rel;
        RREV = Vz_rel/d_ceil;
        OFx = -Vy_rel/d_ceil;
        OFy = -Vx_rel/d_ceil;


    }

    float OF_SensorPlugin::GaussianKernel(double mu, double sigma)
    {
        
        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        
        std::normal_distribution<float> distribution(mu,sigma);
        
        float X = distribution(generator);

        return X;
    }


    GZ_REGISTER_MODEL_PLUGIN(OF_SensorPlugin);
}