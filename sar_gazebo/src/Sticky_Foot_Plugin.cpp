#include <iostream>
#include <Sticky_Foot_Plugin.h>

/*
    This plugin is responsible for joining foot pads to whatever entity they collide with (e.g. ground or ceiling).
*/

namespace gazebo{

    // INITIALIZE PLUGIN AND LOAD VALUES FROM SDF
    void GazeboStickyFoot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
    {
        // SET INITIAL VALUES FOR PARAMETERS
        sticky_flag = false;
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
            gzerr << "[Sticky_Foot_Plugin] Couldn't find specified link " << padName << std::endl;


        // CREATE JOINT NAME
        PAD_NUMBER = _sdf->GetElement("padNumber")->Get<int>(); // Convert SDF element to int
        gzmsg << "\t Pad Number: \t" << PAD_NUMBER << std::endl;
        jointName = "pad_" + std::to_string(PAD_NUMBER) + "_sticky_joint";
        gzmsg << "\t Joint Name:\t" << jointName << std::endl;

        // INITIALIZE SERVICE TO ACTIVATE STICKY PAD
        serviceName = "/activate_Sticky_Pad_" + std::to_string(PAD_NUMBER);
        stickyService = nh.advertiseService(serviceName, &GazeboStickyFoot::Activate_Sticky_Pads, this);

        // SOMETHING ABOUT CREATING A NAMESPACE ("/"" PREFIX FOR GZTOPICS)
        if (_sdf->HasElement("robotNamespace"))
        {
            namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        }
        else
            gzerr << "[Sticky_Foot_Plugin] Please specify a robotNamespace.\n";

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        // NUMBER OF COLLISION OBJECTS IN THE LINK (Typically one per object)
        unsigned int collision_count = padLink_ptr->GetChildCount();

        // CREATE A MAP OF COLLISION NAMES (KEY) TO COLLISION ENTITIES (VALUES)
        // This is similar to a python dictionary {Key:Value}
        std::map<std::string, physics::CollisionPtr> collisions;
        for (unsigned int i = 0; i < collision_count; i++)
        {
            physics::CollisionPtr collision_entity = padLink_ptr->GetCollision(i); // Create ptr to collision entity in SDF
            gzmsg << "\t Collision Entity: " << collision_entity->GetScopedName().c_str() << std::endl;

            collisions[collision_entity->GetScopedName()] = collision_entity; // Add ptr to mapped collision name
        }

        // SUBSCRIBE TO collisionS FROM GAZEBO
        physics_engine_ = world_->Physics();
        contact_manager_ = physics_engine_->GetContactManager();
        if (!collisions.empty()) // If collision entities exist in the link
        {
            // CREATE A GZ-PUBLISHER THAT PUBLISHES CONTACTS ASSOCIATED TO THE INPUTED COLLISION ENTITIES
            contact_pub_topic = contact_manager_->CreateFilter(padName, collisions); // Returns string of publisher topic

            // CREATE A GZ-SUBSCRIBER THAT LISTENS TO THE ABOVE PUBLISHER AND ACTIVATES A CALLBACK
            contact_sub_ = node_handle_->Subscribe(contact_pub_topic, &GazeboStickyFoot::ContactCallback, this);
            gzmsg << "\t Contact_Sub subscribed to: " << contact_pub_topic.c_str() << std::endl;
        }
        std::cout << "\n\n";
    }

    // CALLBACK ACTIVATED WHENEVER CONTACT DETECTED (MSG RECEIVED FROM GZ-PUB)
    void GazeboStickyFoot::ContactCallback(ConstContactsPtr &msg)
    {
        if (sticky_flag == true && contactLink_ptr == NULL) // If sticky activated and link2 not created yet
        {
            for (int i = 0; i < msg->contact_size(); i++)
            {

                // GET LINK NAME FOR COLLISION 2 ENTITY (ceiling)
                // I am assuming collision2 will always be the other object and not the pad
                contactLink_ptr = boost::dynamic_pointer_cast<physics::Collision>(world_->EntityByName(msg->contact(i).collision2().c_str()))->GetLink();

                // IF JOINT DOESN'T ALREADY EXIST CREATE IT
                if (joint_ptr == NULL)
                {
                    joint_ptr = model_->CreateJoint(jointName, "fixed", padLink_ptr, contactLink_ptr);
                    printf("[Pad_%d]: Joint Created\t(%s->%s)\n", PAD_NUMBER, padLink_ptr->GetName().c_str(), contactLink_ptr->GetName().c_str());

                    // PUBLISH EACH TIME A PAD IS CONNECTS TO THE CEILING
                    switch(PAD_NUMBER)
                    {
                        case 1:
                            Sticky_Pad_Connect_msg.Pad1_Contact = 1;
                            break;
                        case 2:
                            Sticky_Pad_Connect_msg.Pad2_Contact = 1;
                            break;
                        case 3:
                            Sticky_Pad_Connect_msg.Pad3_Contact = 1;
                            break;
                        case 4:
                            Sticky_Pad_Connect_msg.Pad4_Contact = 1;
                            break;
                    }

                    Sticky_Pad_Connect_Publisher.publish(Sticky_Pad_Connect_msg);
                }

                break;
            }
        }
    }

    // ROS SERVICE CALLBACK TO ACTIVATE/DEACTIVATE STICKY FOOT BEHAVIOR
    bool GazeboStickyFoot::Activate_Sticky_Pads(sar_msgs::Activate_Sticky_Pads::Request &req, sar_msgs::Activate_Sticky_Pads::Response &res)
    {
        if (req.stickyFlag == false && sticky_flag == true) // TURN OFF STICKY FOOT
        {
            sticky_flag = false;
            printf("[Pad_%d]: Sticky_Disabled\n", PAD_NUMBER);

            if (joint_ptr != NULL)
            {
                model_->RemoveJoint(jointName);
                printf("[Pad_%d]: Joint Removed\t(%s->%s)\n", PAD_NUMBER, padLink_ptr->GetName().c_str(), contactLink_ptr->GetName().c_str());
            }

            if (PAD_NUMBER == 4)
            {
                printf("\n"); // Add extra space to break up console output
            }

            // RESET JOINT AND CONTACT LINK POINTERS
            joint_ptr = NULL;
            contactLink_ptr = NULL;
        }
        if (req.stickyFlag == true && sticky_flag == false) // TURN ON STICKY FOOT
        {
            sticky_flag = true;
            printf("[Pad_%d]: Sticky_Enabled\n", PAD_NUMBER);
            if (PAD_NUMBER == 4)
            {
                printf("\n"); // Add extra space to break up console output
            }
        }

        res.cmd_Success = true; // TODO: This would be a good feature to check that everything worked alright
        return true;            // I'm not sure where this goes
    }

GZ_REGISTER_MODEL_PLUGIN(GazeboStickyFoot);
}