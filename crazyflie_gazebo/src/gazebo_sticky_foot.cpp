#include <iostream>
#include <gazebo_sticky_foot.h>

namespace gazebo{


// This gets called when model is loaded and pulls values from sdf file
void GazeboStickyFoot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
{
    gzmsg<<"!!!!! Entering GazeboStickyFoot::Load !!!!!\n";
    model_ = _model;
    world_ = model_->GetWorld();
    //model_name_ = model_->GetName();

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    //updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboStickyFoot::OnUpdate, this, _1));

    // INITIALIZE SOME SORT OF GAZEBO SUBSCRIBER
    getSdfParam<std::string>(_sdf, "stickyEnableSubTopic", sticky_enable_sub_topic_, sticky_enable_sub_topic_);
    sticky_enable_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + sticky_enable_sub_topic_, &GazeboStickyFoot::StickyEnableCallback, this);

    // GRAB SELECTED LINK FROM SDF
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>(); // Pad_1
    gzmsg<<"!!!!! link_name_ = "<<link_name_<<"\n";
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzerr<<"[gazebo_sticky_foot] Couldn't find specified link " << link_name_ << std::endl;

    // SET INITIAL VALUES FOR PARAMETERS
    sticky_ = false;    // Have the sticky feature turned off by default
    link2_ = NULL;      // Link that pad will joint to at contact
    joint_ = NULL;      // The future joint between the links

    // EXTRACT OTHER VALUES
    vz_max_ = _sdf->GetElement("maxUpVelocity")->Get<double>(); // I'm not sure on this part
    pad_number_ = _sdf->GetElement("padNumber")->Get<int>();
    joint_name_ = "pad_" + std::to_string(pad_number_) + "_sticky_joint";
    std::cout<<"Joint name of pad_"<<pad_number_<<" is "<<joint_name_<<std::endl;



    // EXTRACT NAMES OF ALL COLLISION OBJECTS IN THE LINK
    unsigned int collsion_count = link_->GetChildCount(); // ccount = child count

    // CREATE A MAP OF COLLISION NAMES (KEY) TO COLLISION ENTITIES (VALUES)
    // This is similar to a python dictionary {Key:Value}
    std::map<std::string, physics::CollisionPtr> collisions;
    for(unsigned int i = 0; i < collsion_count; i++)
    {
        physics::CollisionPtr collision_entity = link_->GetCollision(i);
        std::cout << "[gazebo_sticky_foot] Collision Entity: " << collision_entity->GetScopedName().c_str() << std::endl;

        // I'm not sure what was referenced here? 
        /*
        //What even is the point of this check? It SEEMS to look for the same collision appearing twice, but how would that even happen?
        std::map<std::string, physics::CollisionPtr>::iterator collIter = this->dataPtr->collisions.find(collision->GetScopedName());
        if (collIter != this->dataPtr->collisions.end()) continue;
        */


        collisions[collision_entity->GetScopedName()] = collision_entity;
    }
    
    /* FAILED CODE?
    contact_node_ = transport::NodePtr(new transport::Node());
    contact_node_->Init(world_->GetName());
    */

    // SUBSCRIBE TO COLLSIONS FROM GAZEBO
    physics_engine_ = world_->Physics();
    contact_manager_ = physics_engine_->GetContactManager();
    if (!collisions.empty()) // If collsions entities exist in the link
    {
        // CREATE A GZ-PUBLISHER THAT PUBLISHES CONTACTS ASSOCIATED TO THE INPUTED COLLISION ENTITIES
        contact_sub_topic_ = contact_manager_->CreateFilter(link_name_, collisions);
        std::cout<<"[gazebo_sticky_foot]: ContactCB subscribed to: "<<contact_sub_topic_.c_str()<<std::endl;
        
        // CREATE A GZ-SUBSCRIBER THAT LISTENS TO THE ABOVE PUBLISHER AND ACTIVATES A CALLBACK
        contact_sub_ = node_handle_->Subscribe(contact_sub_topic_, &GazeboStickyFoot::ContactCB, this);
    }

    /* FAILED CODE?
    joint_ = physics_engine_->CreateJoint("fixed", model_);
    //joint_->SetName(model_->GetName() + "__sticking_joint__");
    joint_->SetName("__sticking_joint__");
    */


}

/*
// This gets called by the world update start event.
void GazeboStickyFoot::OnUpdate(const common::UpdateInfo& _info)
{
}
*/

// This gets called when motorspeed callback return negative number
// This uses gz topics which use some protobuff thing different than ROS framework
void GazeboStickyFoot::StickyEnableCallback(CommandMotorSpeedPtr &rot_velocities)
{
    //model_ = world_->ModelByName(model_name_);
    

    // IF MS COMMAND = [-X,0,0,0] TURN OFF STICKY
    if(rot_velocities->motor_speed(1) < 0.5)
    {
        // "link_"  WILL HAVE NAMES pad_[X]
        std::cout<<link_->GetName().c_str()<< " NOW NOT STICKY "<< std::endl;
        sticky_ = false;
        //joint_->Detach();     // Detach() doesn't work, don't know why
        if (joint_ != NULL)
        {
            if (model_->RemoveJoint(joint_name_))
                std::cout<<"Joint (" <<joint_->GetName().c_str() << ") removed successufully"<<std::endl;
                // "joint_" WILL HAVE NAME "pad_[X]_sticky_joint"
                

            else
                std::cout<<"Joint removed failed"<<std::endl;
        }
        joint_ = NULL;
        link2_ = NULL;
        
    }
    else // IF MS COMMAND = [-X,1,0,0] TURN ON STICKY
    {
        std::cout<<link_->GetName().c_str()<< " NOW STICKY "<< std::endl;
        sticky_ = true;

        

        
    }
}

// This gets called when contact detected (Not sure where this is called)
void GazeboStickyFoot::ContactCB(ConstContactsPtr &msg)
{
    if(sticky_ && link2_==NULL)
    {
        for(int i = 0; i < msg->contact_size(); i++)
        {
            physics::LinkPtr candidate = NULL;
            if(strcmp(msg->contact(i).collision1().c_str(), link_name_.c_str()))
            {
                candidate = boost::dynamic_pointer_cast<physics::Collision>( world_->EntityByName(msg->contact(i).collision1().c_str()) )->GetLink();
            }
            if(strcmp(msg->contact(i).collision2().c_str(), link_name_.c_str()))
            {
                candidate = boost::dynamic_pointer_cast<physics::Collision>( world_->EntityByName(msg->contact(i).collision2().c_str()) )->GetLink();
            }
            if(candidate != NULL)
            {
                //if(candidate->GetInertial()->GetMass() <= this->max_mass)
                //{
                    std::cout<<"[gazebo_sticky_foot:] "<<link_->GetName().c_str()<<" grabbing link "<<candidate->GetName().c_str()<<std::endl;

                    link2_ = candidate;
                    
                    if (joint_==NULL)
                        joint_ = model_->CreateJoint(joint_name_, "fixed", link_, link2_);
                        std::cout<<"Joint (" <<joint_->GetName().c_str() << ")"<<std::endl;

                        // PUBLISH EACH TIME PAD IS CONNECTED TO THE CEILING
                        crazyflie_msgs::PadConnect msg;
                        std::string str = joint_->GetName().c_str();    // Get joint name as string
                        str = str.substr(4,1);                          // Trim to pad index
                        msg.Pad_Num = atoi(str.c_str())+1;              // Convert to pad number
                        PadConnect_Publisher.publish(msg);

                    //Attach the joint
                    joint_->Load(link_, link2_, ignition::math::Pose3d());
                    joint_->Attach(link_, link2_);

                    break;
                //}
            }
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboStickyFoot);
}