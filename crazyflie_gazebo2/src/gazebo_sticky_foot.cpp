#include <iostream>
#include <gazebo_sticky_foot.h>

/*
    This plugin is responsible for joining foot pads to whatever entity they collide with (e.g. ground or ceiling).
*/

namespace gazebo{


    // This gets called when model is loaded and pulls values from sdf file
    void GazeboStickyFoot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
        // SET INITIAL VALUES FOR PARAMETERS
        sticky_ = false;
        contactLink_ptr = NULL; // Link that pad will join to at contact
        joint_ptr = NULL;       // The future joint between contacting links

        gzmsg << "Loading GazeboStickyFoot Plugin\n";
        model_ = _model;
        world_ = model_->GetWorld();

        // GRAB SELECTED LINK FROM SDF
        padName = _sdf->GetElement("linkName")->Get<std::string>(); // Pad_1
        gzmsg << "\t Pad Name:\t" << padName << std::endl;
        padLink_ptr = model_->GetLink(padName); // Returns a ptr to link
        if (padLink_ptr == NULL)
            gzerr << "[gazebo_sticky_foot] Couldn't find specified link " << padName << std::endl;


        // CREATE JOINT NAME
        pad_number_ = _sdf->GetElement("padNumber")->Get<int>(); // Convert SDF element to int
        gzmsg << "\t Pad Number: \t" << pad_number_ << std::endl;
        jointName = "pad_" + std::to_string(pad_number_) + "_sticky_joint";
        gzmsg << "\t Joint Name:\t" << jointName << std::endl;

        serviceName = "/activate_Sticky_Pad_" + std::to_string(pad_number_);
        stickyService = nh.advertiseService(serviceName, &GazeboStickyFoot::activateSticky, this);

        // SOMETHING ABOUT CREATING A NAMESPACE ("/"" PREFIX FOR GZTOPICS)
        if (_sdf->HasElement("robotNamespace"))
        {
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        else
            gzerr << "[gazebo_sticky_foot] Please specify a robotNamespace.\n";

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);
    }

    bool GazeboStickyFoot::activateSticky(crazyflie_msgs::activateSticky::Request &req, crazyflie_msgs::activateSticky::Response &res)
    {
        return true;
    }

GZ_REGISTER_MODEL_PLUGIN(GazeboStickyFoot);
}