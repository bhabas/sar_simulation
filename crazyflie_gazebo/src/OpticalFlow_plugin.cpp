#include "OpticalFlow_plugin.h"



namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void OpticalFlow_plugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading OpticalFlow_plugin\n";
        model_ptr = parent;

        // COLLECT PARAMS FROM SDF
        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_ptr->GetLink(linkName);

        topicName = _sdf->GetElement("topicName")->Get<std::string>();
        updateRate = _sdf->GetElement("updateRate")->Get<int>();

        Tau_gaussianNoise = _sdf->GetElement("Tau_gaussianNoise")->Get<double>();
        OFx_gaussianNoise = _sdf->GetElement("OFx_gaussianNoise")->Get<double>();



        // LOAD PLANE LOCATION
        ros::param::get("/PLANE_SETTINGS/Plane_Config",Plane_Config);
        ros::param::get("/Plane_Config/" + Plane_Config + "/Pos_X",r_PO.X());
        ros::param::get("/Plane_Config/" + Plane_Config + "/Pos_Y",r_PO.Y());
        ros::param::get("/Plane_Config/" + Plane_Config + "/Pos_Z",r_PO.Z());
        ros::param::get("/Plane_Config/" + Plane_Config + "/Plane_Angle",Plane_Angle);
        Plane_Angle = Plane_Angle*M_PI/180; // Convert to radians

        // DEFINE PLANE NORMAL UNIT-VECTOR
        n_hat.X() = sin(Plane_Angle);
        n_hat.Y() = 0;
        n_hat.Z() = -cos(Plane_Angle);

        // DEFINE PLANE TANGENT UNIT-VECTOR
        t_x.X() = -cos(Plane_Angle);
        t_x.Y() = 0;
        t_x.Z() = -sin(Plane_Angle);






        // INIT PUBLISHER AND PUBLISHING THREAD
        OF_Publisher = nh.advertise<crazyflie_msgs::OF_SensorData>(topicName,1);
        publisherThread = std::thread(&OpticalFlow_plugin::Publish_OF_Data, this);

        
        gzmsg << "\t Loading Completed" << std::endl;
        std::cout << "\n\n";

    }



    void OpticalFlow_plugin::Publish_OF_Data()
    {
        ros::Rate rate(updateRate);
        while(ros::ok)
        {
            // UPDATE POS AND VEL
            r_BO = link_ptr->WorldPose().Pos();
            V_BO = link_ptr->WorldLinearVel();

            // CALC DISPLACEMENT FROM PLANE CENTER
            r_PB = r_PO - r_BO; 

            // CALC RELATIVE DISTANCE AND VEL
            D_perp = r_PB.Dot(n_hat);
            V_perp = V_BO.Dot(n_hat) + 1e-3;
            V_tx = V_BO.Dot(t_x) + 1e-3;

            if (abs(D_perp) < 0.02)
            {
                D_perp = 0.0;
            }

            // CALC OPTICAL FLOW VALUES
            Tau = D_perp/V_perp;
            OFx = -V_tx/D_perp;

            // PUBLISH OPTICAL FLOW VALUES
            Tau = boost::algorithm::clamp(Tau,0.0,5.0);
            OFx = boost::algorithm::clamp(OFx,-50,50);


            OF_Data_msg.Tau = Tau + GaussianKernel(0,Tau_gaussianNoise);
            OF_Data_msg.OFx = OFx + GaussianKernel(0,OFx_gaussianNoise);
            OF_Data_msg.d_ceil = D_perp; // Change value to d_perp

            OF_Publisher.publish(OF_Data_msg);
            rate.sleep();
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