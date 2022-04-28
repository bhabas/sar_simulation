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
        printf("[DOMAIN RAND PLUGIN] Inertia Values Updated\n");
        inertia_ptr->SetMass(req.mass);
        inertia_ptr->SetInertiaMatrix(
            req.Inertia.x,
            req.Inertia.y,
            req.Inertia.z,
            0.0,
            0.0,
            0.0);

        link_ptr->UpdateMass();
        link_ptr->Update();

        res.srv_Success = true;
        return true;
    }


    GZ_REGISTER_MODEL_PLUGIN(DomainRand_plugin);
}