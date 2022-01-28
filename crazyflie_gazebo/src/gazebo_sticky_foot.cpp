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
    //model_name_ = model_->GetName();


    // GRAB SELECTED LINK FROM SDF
    padName = _sdf->GetElement("linkName")->Get<std::string>(); // Pad_1
    gzmsg<<"\t Pad Name:\t"<<padName<<std::endl;
    padLink_ptr = model_->GetLink(padName); // Returns a ptr to link
    if (padLink_ptr == NULL)
        gzerr<<"[gazebo_sticky_foot] Couldn't find specified link " << padName << std::endl;


    // CREATE JOINT NAME
    pad_number_ = _sdf->GetElement("padNumber")->Get<int>(); // Convert SDF element to int
    jointName = "pad_" + std::to_string(pad_number_) + "_sticky_joint";
    gzmsg<<"\t Joint Name:\t"<<jointName<<std::endl;


    // SOMETHING ABOUT CREATING A NAMESPACE ("/"" PREFIX FOR GZTOPICS)
    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    else
        gzerr << "[gazebo_sticky_foot] Please specify a robotNamespace.\n";

    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);


    // NUMBER OF COLLISION OBJECTS IN THE LINK (Typically one per object)
    unsigned int collsion_count = padLink_ptr->GetChildCount();

    // CREATE A MAP OF COLLISION NAMES (KEY) TO COLLISION ENTITIES (VALUES)
    // This is similar to a python dictionary {Key:Value}
    std::map<std::string, physics::CollisionPtr> collisions;
    for(unsigned int i = 0; i < collsion_count; i++)
    {
        physics::CollisionPtr collision_entity = padLink_ptr->GetCollision(i); // Create ptr to collision entity in SDF
        gzmsg << "\t Collision Entity: " << collision_entity->GetScopedName().c_str() << std::endl;

        collisions[collision_entity->GetScopedName()] = collision_entity; // Add ptr to mapped collision name
    }
    


    // SUBSCRIBE TO COLLSIONS FROM GAZEBO
    physics_engine_ = world_->Physics();
    contact_manager_ = physics_engine_->GetContactManager();
    if (!collisions.empty()) // If collsion entities exist in the link
    {
        // CREATE A GZ-PUBLISHER THAT PUBLISHES CONTACTS ASSOCIATED TO THE INPUTED COLLISION ENTITIES
        contact_pub_topic = contact_manager_->CreateFilter(padName, collisions); // Returns string of publisher topic
        
        // CREATE A GZ-SUBSCRIBER THAT LISTENS TO THE ABOVE PUBLISHER AND ACTIVATES A CALLBACK
        contact_sub_ = node_handle_->Subscribe(contact_pub_topic, &GazeboStickyFoot::ContactCallback, this);
        gzmsg<<"\t Contact_Sub subscribed to: "<<contact_pub_topic.c_str()<<std::endl;
    }
    std::cout << "\n\n";

}




// CALLBACK ACTIVATED WHENEVER CONTACT DETECTED (MSG RECEIVED FROM GZ-PUB)
void GazeboStickyFoot::ContactCallback(ConstContactsPtr &msg)
{
    if(sticky_==true && contactLink_ptr==NULL) // If sticky activated and link2 not created yet
    {
        for(int i = 0; i < msg->contact_size(); i++)
        {
            
            
            // GET LINK NAME FOR COLLISION 1 ENTITY (Pad_x)
            // candidate = boost::dynamic_pointer_cast<physics::Collision>( world_->EntityByName(msg->contact(i).collision1().c_str()) )->GetLink();
            
            // GET LINK NAME FOR COLLISION 2 ENTITY (ceiling)
            // I am assuming collision2 will always be the other object and not the pad
            contactLink_ptr = boost::dynamic_pointer_cast<physics::Collision>( world_->EntityByName(msg->contact(i).collision2().c_str()) )->GetLink();

            // IF JOINT DOESN'T ALREADY EXIST CREATE IT
            if (joint_ptr == NULL)
                joint_ptr = model_->CreateJoint(jointName, "fixed", padLink_ptr, contactLink_ptr);
                printf("[Pad_%d]: Joint Created\t(%s->%s)\n",pad_number_,padLink_ptr->GetName().c_str(),contactLink_ptr->GetName().c_str());               


                // PUBLISH EACH TIME A PAD IS CONNECTS TO THE CEILING
                crazyflie_msgs::PadConnect msg;
                msg.Pad_Num = pad_number_;                // Convert to pad number
                PadConnect_Publisher.publish(msg);

            break;


        }
    }
}

bool GazeboStickyFoot::callback_reset_counter(crazyflie_msgs::AddTwoInts::Request &req, crazyflie_msgs::AddTwoInts::Response &res)
{
    std::cout << pad_number_ << std::endl;
    return true;
}

void GazeboStickyFoot::RLCmdCallback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
{

    // CREATE CMD VECTOR AND VALS FROM SUBSCRIBED MESSAGE
    int cmd_type = msg->cmd_type;       // Read cmd type from incoming message
    int cmd_flag = (int)msg->cmd_flag;  // Construct flag from cmd flag value

    if(cmd_type == 11)
    {
        
        if(cmd_flag == 0 && sticky_) // TURN OFF STICKY FOOT
        {
            sticky_ = false;
            printf("[Pad_%d]: Sticky_Disabled\n",pad_number_);


            if (joint_ptr != NULL)
            {
                model_->RemoveJoint(jointName);
                printf("[Pad_%d]: Joint Removed\t(%s->%s)\n",pad_number_,padLink_ptr->GetName().c_str(),contactLink_ptr->GetName().c_str());
            }

            // RESET JOINT AND CONTACT LINK POINTERS
            joint_ptr = NULL;
            contactLink_ptr = NULL;
        }
        if(cmd_flag == 1 && !sticky_) // TURN ON STICKY FOOT
        {
            sticky_ = true;
            printf("[Pad_%d]: Sticky_Enabled\n",pad_number_);
        }
    }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboStickyFoot);
}