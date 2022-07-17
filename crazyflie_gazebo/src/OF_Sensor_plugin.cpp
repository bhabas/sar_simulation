#include "OF_Sensor_plugin.h"



namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void OF_SensorPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading OF_SensorPlugin\n";
        model_ptr = parent;

        // COLLECT PARAMS FROM SDF
        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_ptr->GetLink(linkName);

        topicName = _sdf->GetElement("topicName")->Get<std::string>();
        updateRate = _sdf->GetElement("updateRate")->Get<int>();

        Tau_gaussianNoise = _sdf->GetElement("Tau_gaussianNoise")->Get<double>();
        OFx_gaussianNoise = _sdf->GetElement("OFx_gaussianNoise")->Get<double>();
        OFy_gaussianNoise = _sdf->GetElement("OFy_gaussianNoise")->Get<double>();

        // GET CEILING HEIGHT FROM ROS PARAM
        ros::param::get("/CEILING_HEIGHT",_H_CEILING);

        // INIT PUBLISHER AND PUBLISHING THREAD
        OF_Publisher = nh.advertise<crazyflie_msgs::OF_SensorData>(topicName,1);
        publisherThread = std::thread(&OF_SensorPlugin::Publish_OF_Data, this);

        
        gzmsg << "\t Loading Completed" << std::endl;
        std::cout << "\n\n";
    }



    void OF_SensorPlugin::Publish_OF_Data()
    {
        ros::Rate rate(updateRate);
        while(ros::ok)
        {
        
            // DEFINE VELOCITIES RELATIVE TO BODY ORIENTATION
            Vx_rel = link_ptr->RelativeLinearVel().X();
            Vy_rel = link_ptr->RelativeLinearVel().Y();
            Vz_rel = link_ptr->RelativeLinearVel().Z();

            // CALCULATE OPTICAL FLOW VALUES
            d_ceil = _H_CEILING - link_ptr->WorldPose().Z();
            if (d_ceil < 0.02)
            {
                d_ceil = 0.0;
            }

            Tau = d_ceil/(Vz_rel + 1e-3f); 
            OFx = -Vy_rel/(d_ceil + 1e-3f); // + Angular velocity values
            OFy = -Vx_rel/(d_ceil + 1e-3f); // + Angular velocity values

            // PUBLISH OPTICAL FLOW VALUES
            Tau = boost::algorithm::clamp(Tau,0.0,10.0);

            OF_Data_msg.Tau = Tau + GaussianKernel(0,Tau_gaussianNoise);
            OF_Data_msg.OFx = OFx + GaussianKernel(0,OFx_gaussianNoise);
            OF_Data_msg.OFy = OFy + GaussianKernel(0,OFy_gaussianNoise);
            OF_Data_msg.d_ceil = d_ceil;

            OF_Publisher.publish(OF_Data_msg);
            rate.sleep();
        }
    }


    float OF_SensorPlugin::GaussianKernel(double mu, double sigma)
    {
        // PULL VALUE FROM THE GIVEN GAUSSIAN
        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        
        std::normal_distribution<float> distribution(mu,sigma);
        
        float X = distribution(generator);

        return X;
    }


    GZ_REGISTER_MODEL_PLUGIN(OF_SensorPlugin);
}