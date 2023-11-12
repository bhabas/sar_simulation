#include "OpticalFlow_plugin.h"
#include <iostream>
#include <chrono> 
using namespace std;


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

        Theta_x_gaussianNoise = _sdf->GetElement("Theta_x_gaussianNoise")->Get<double>();
        Theta_y_gaussianNoise = _sdf->GetElement("Theta_y_gaussianNoise")->Get<double>();
        Theta_z_gaussianNoise = _sdf->GetElement("Theta_z_gaussianNoise")->Get<double>();


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

        // DEFINE PLANE TANGENT UNIT-VECTOR
        t_y.X() = 0;
        t_y.Y() = 1;
        t_y.Z() = 0;



        // INIT PUBLISHER AND PUBLISHING THREAD
        OF_Publisher = nh.advertise<crazyflie_msgs::OF_SensorData>(topicName,1);
        publisherThread = std::thread(&OpticalFlow_plugin::Publish_OF_Data, this);

        
        gzmsg << "\t Loading Completed" << std::endl;
        std::cout << "\n\n";

    }



    void OpticalFlow_plugin::Publish_OF_Data()
    {
        ros::Rate rate(updateRate);
        // int i = 0;
        // auto t1 = std::chrono::high_resolution_clock::now();
        while(ros::ok)
        {
            // UPDATE POS AND VEL
            r_BO = link_ptr->WorldPose().Pos();
            V_BO = link_ptr->WorldLinearVel();

            // CALC DISPLACEMENT FROM PLANE CENTER
            r_PB = r_PO - r_BO; 

            // CALC RELATIVE DISTANCE AND VEL
            D_perp = r_PB.Dot(n_hat) + 1e-6;
            V_perp = V_BO.Dot(n_hat);
            V_tx = V_BO.Dot(t_x);
            V_ty = V_BO.Dot(t_y);

            if (abs(D_perp) < 0.02)
            {
                D_perp = 0.0;
            }

            // CALC OPTICAL FLOW VALUES
            Theta_z = V_perp/D_perp + GaussianKernel(0,Theta_z_gaussianNoise);
            Theta_x = V_tx/D_perp + GaussianKernel(0,Theta_x_gaussianNoise);
            Theta_y = V_ty/D_perp + GaussianKernel(0,Theta_y_gaussianNoise);
            
            // if (i<1000) {
            // cout << "Theta_x_Noisy: " << Theta_x << endl;
            // cout << "Theta_x: " << V_tx/D_perp << endl;
            // cout << "Theta_y_Noisy: " << Theta_y << endl;
            // cout << "Theta_y: " << V_ty/D_perp << endl;
            // cout << "Theta_z_Noisy: " << Theta_z << endl;
            // cout << "Theta_z: " << V_perp/D_perp << endl;
            // i++;
            // auto t2 = std::chrono::high_resolution_clock::now();
            // std::cout << "time " << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << endl;
            // }

            // CLAMP OPTICAL FLOW VALUES
            Theta_x = boost::algorithm::clamp(Theta_x,-50,50);
            Theta_y = boost::algorithm::clamp(Theta_y,-50,50);
            Theta_z = boost::algorithm::clamp(Theta_z,0,50);
            Tau = boost::algorithm::clamp(1/Theta_z,0,5);;

            // PUBLISH OPTICAL FLOW VALUES
            OF_Data_msg.Tau = Tau;
            OF_Data_msg.Theta_y = Theta_x;
            OF_Data_msg.Theta_x = Theta_y;
            OF_Data_msg.D_perp = D_perp; 
            
            OF_Data_msg.Theta_x = Theta_x;
            OF_Data_msg.Theta_y = Theta_y;
            OF_Data_msg.Theta_z = Theta_z;
            OF_Data_msg.D_perp = D_perp;



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