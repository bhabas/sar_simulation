#include <iostream>
#include <gazebo_sticky_foot.h>

/*
    (LEGACY CODE)
    This plugin is responsible for joining foot pads to whatever entity they collide with (e.g. ground or ceiling).
    It turns on/off through the motorspeed commands where a negative first value sends the command here and the 
    next value is a bool turning sticky on/off. 
*/

namespace gazebo{


// This gets called when model is loaded and pulls values from sdf file
void GazeboStickyFoot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
{
    gzmsg << "Loading GazeboStickyFoot Plugin\n";
    model_ = _model;
    world_ = model_->GetWorld();
    //model_name_ = model_->GetName();

    // GRAB SELECTED LINK FROM SDF
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>(); // Pad_1
    gzmsg<<"\t link_name_ = "<<link_name_<<std::endl;
    link_ = model_->GetLink(link_name_); // Returns a ptr to link
    if (link_ == NULL)
        gzerr<<"[gazebo_sticky_foot] Couldn't find specified link " << link_name_ << std::endl;


    // CREATE JOINT NAME
    pad_number_ = _sdf->GetElement("padNumber")->Get<int>(); // Convert SDF element to int
    joint_name_ = "pad_" + std::to_string(pad_number_) + "_sticky_joint";
    gzmsg<<"\t joint_name_= "<<joint_name_<<std::endl;


    // SOMETHING ABOUT CREATING A NAMESPACE ("/"" PREFIX FOR GZTOPICS)
    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    else
        gzerr << "[gazebo_sticky_foot] Please specify a robotNamespace.\n";

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);



    // SET INITIAL VALUES FOR PARAMETERS
    sticky_ = false;    // Have the sticky feature turned off by default
    link2_ = NULL;      // Link that pad will joint to at contact
    joint_ = NULL;      // The future joint between the links



    // NUMBER OF COLLISION OBJECTS IN THE LINK
    unsigned int collsion_count = link_->GetChildCount();

    // CREATE A MAP OF COLLISION NAMES (KEY) TO COLLISION ENTITIES (VALUES)
    // This is similar to a python dictionary {Key:Value}
    std::map<std::string, physics::CollisionPtr> collisions;
    for(unsigned int i = 0; i < collsion_count; i++)
    {
        physics::CollisionPtr collision_entity = link_->GetCollision(i); // Create ptr to collision entity in SDF
        gzmsg << "\t Collision Entity: " << collision_entity->GetScopedName().c_str() << std::endl;

        collisions[collision_entity->GetScopedName()] = collision_entity; // Add ptr to mapped collision name
    }
    


    // SUBSCRIBE TO COLLSIONS FROM GAZEBO
    physics_engine_ = world_->Physics();
    contact_manager_ = physics_engine_->GetContactManager();
    if (!collisions.empty()) // If collsion entities exist in the link
    {
        // CREATE A GZ-PUBLISHER THAT PUBLISHES CONTACTS ASSOCIATED TO THE INPUTED COLLISION ENTITIES
        contact_pub_topic = contact_manager_->CreateFilter(link_name_, collisions); // Returns string of publisher topic
        
        // CREATE A GZ-SUBSCRIBER THAT LISTENS TO THE ABOVE PUBLISHER AND ACTIVATES A CALLBACK
        contact_sub_ = node_handle_->Subscribe(contact_pub_topic, &GazeboStickyFoot::ContactCallback, this);
        gzmsg<<"\t Contact_Sub subscribed to: "<<contact_pub_topic.c_str()<<std::endl;
    }
    std::cout << "\n\n";

}




// CALLBACK ACTIVATED WHENEVER CONTACT DETECTED (MSG RECEIVED FROM GZ-PUB)
void GazeboStickyFoot::ContactCallback(ConstContactsPtr &msg)
{
    if(sticky_==true && link2_==NULL) // If sticky activated and link2 not created yet
    {
        for(int i = 0; i < msg->contact_size(); i++)
        {
            

            physics::LinkPtr candidate = NULL;
            
            // GET LINK NAME FOR COLLISION 1 ENTITY (Pad_x)
            // candidate = boost::dynamic_pointer_cast<physics::Collision>( world_->EntityByName(msg->contact(i).collision1().c_str()) )->GetLink();
            
            // GET LINK NAME FOR COLLISION 2 ENTITY (ceiling)
            link2_ = boost::dynamic_pointer_cast<physics::Collision>( world_->EntityByName(msg->contact(i).collision2().c_str()) )->GetLink();

            std::cout<<"[gazebo_sticky_foot:] Link: "<<link_->GetName().c_str()<<" grabbing Link: "<<link2_->GetName().c_str()<<std::endl;

            // IF JOINT DOESN'T ALREADY EXIST CREATE IT
            if (joint_== NULL)
                joint_ = model_->CreateJoint(joint_name_, "fixed", link_, link2_);
                std::cout << "[gazebo_sticky_foot:] Joint Created: (" <<joint_->GetName().c_str() << ")"<<std::endl;

                // PUBLISH EACH TIME PAD IS CONNECTED TO THE CEILING
                crazyflie_msgs::PadConnect msg;
                std::string str = joint_->GetName().c_str();    // Get joint name as string
                str = str.substr(4,1);                          // Trim to pad index
                msg.Pad_Num = atoi(str.c_str())+1;              // Convert to pad number
                PadConnect_Publisher.publish(msg);

            // ATTACH THE JOINT
            joint_->Load(link_, link2_, ignition::math::Pose3d());
            joint_->Attach(link_, link2_);

            break;


        }
    }
}

void GazeboStickyFoot::RLCmdCallback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{

    // CREATE CMD VECTOR AND VALS FROM SUBSCRIBED MESSAGE
    int cmd_type = msg->cmd_type;                       // Read cmd type from incoming message
    const geometry_msgs::Point cmd_vals = msg->cmd_vals;    // Construct vector from cmd values
    float cmd_flag = msg->cmd_flag;                     // Construct flag from cmd flag value

    if(cmd_type == 11)
    {
        if(cmd_flag == 1) // TURN ON STICKY FOOT
        {
            std::cout<<link_->GetName().c_str()<< " NOW STICKY "<< std::endl;
            sticky_ = true;
        }
        if(cmd_flag == 0) // TURN OFF STICKY FOOT
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
                    std::cout<<"Joint (" <<joint_->GetName().c_str() << ") removal failed"<<std::endl;
            }
            joint_ = NULL;
            link2_ = NULL;
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboStickyFoot);
}