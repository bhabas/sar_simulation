#include <iostream>
#include <DomainRandomization.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void DomainRand_plugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading DomainRand Plugin\n";
        model_ptr = parent;

        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_ptr->GetLink(linkName);
        DomainRandService = nh.advertiseService("/CF_Internal/DomainRand", &DomainRand_plugin::UpdateInertia, this);
        inertia_ptr = link_ptr->GetInertial();
        
        printf("\n\n");

    }

    bool DomainRand_plugin::UpdateInertia(crazyflie_msgs::domainRand::Request &req, crazyflie_msgs::domainRand::Response &res)
    {
        printf("ehere\n");
        return true;
    }


    void DomainRand_plugin::RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
    {
        if(msg->cmd_type == 50)
        {
            printf("Domain Rand Plugin\n");
            // inertia_ptr->SetMass(1.0);
            // inertia_ptr->SetInertiaMatrix(
            //     15.83e-6,
            //     17.00e-5,
            //     31.19e-6,
            //     0.0,
            //     0.0,
            //     0.0);
            ignition::math::Pose3d pose(
                ignition::math::Vector3d(0, 0, 0.2),
                ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)
            );
            inertia_ptr->SetCoG(pose);
            link_ptr->UpdateMass();
            link_ptr->Update();
        }
        if(msg->cmd_type == 51)
        {
            // printf("%.3f\n",inertia_ptr->CoG().);
            std::cout << inertia_ptr->CoG() << std::endl;
            // inertia_ptr->UpdateParameters()
        }
        else if(msg->cmd_type == 0)
        {
            ignition::math::Pose3d pose(
                ignition::math::Vector3d(0, 0, 0.5),
                ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)
            );
            model_ptr->SetWorldPose(pose);
            model_ptr->SetWorldTwist(ignition::math::Vector3d(0, 0, 0),ignition::math::Vector3d(0, 0, 0));

        }
    }

    GZ_REGISTER_MODEL_PLUGIN(DomainRand_plugin);
}