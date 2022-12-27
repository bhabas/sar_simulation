#include "OpticalFlow_plugin.h"



namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void OpticalFlow_plugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        
    }



    void OpticalFlow_plugin::Publish_OF_Data()
    {
        ros::Rate rate(updateRate);
        while(ros::ok)
        {
        
            
        }
    }


    float OpticalFlow_plugin::GaussianKernel(double mu, double sigma)
    {
        // PULL VALUE FROM THE GIVEN GAUSSIAN
        unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        
        std::normal_distribution<float> distribution(mu,sigma);
        
        float X = distribution(generator);

        return X;
    }


    GZ_REGISTER_MODEL_PLUGIN(OpticalFlow_plugin);
}