#include <iostream>
#include <Domain_Randomization_Plugin.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void Domain_Randomization_Plugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading DomainRand Plugin\n";
        model_ptr = parent;

        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_ptr->GetLink(linkName);
        DomainRandService = nh.advertiseService("/SAR_Internal/DomainRand", &Domain_Randomization_Plugin::UpdateInertia, this);
        inertia_ptr = link_ptr->GetInertial();
        
        printf("\n\n");

    }

    bool Domain_Randomization_Plugin::UpdateInertia(sar_msgs::domainRand::Request &req, sar_msgs::domainRand::Response &res)
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


    GZ_REGISTER_MODEL_PLUGIN(Domain_Randomization_Plugin);
}