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

        inertia_ptr = link_ptr->GetInertial();
        std::cout << inertia_ptr->Mass() << std::endl;
        
        printf("\n\n");

    }


    void DomainRand_plugin::RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
    {
        if(msg->cmd_type == 50)
        {
            printf("Domain Rand Plugin\n");
            // inertia_ptr->SetMass(1.0);
            inertia_ptr->SetInertiaMatrix(
                15.83e-6,
                17.00e-5,
                31.19e-6,
                0.0,
                0.0,
                0.0);
            // inertia_ptr->SetCoG(0.1,0,0);
            link_ptr->UpdateMass();
        }
        if(msg->cmd_type == 51)
        {
            printf("%.3f\n",inertia_ptr->Mass());
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