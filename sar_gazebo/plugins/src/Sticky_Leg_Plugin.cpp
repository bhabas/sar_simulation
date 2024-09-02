
#include <Sticky_Leg_Plugin.h>

namespace gazebo
{
    void Sticky_Leg_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // LOAD VALUES FROM SDF
        Joint_Name = _sdf->GetElement("JointName")->Get<std::string>();
        Link_Name = _sdf->GetElement("LinkName")->Get<std::string>();
        Leg_Number = _sdf->GetElement("LegNumber")->Get<int>();

        gzmsg << "Loading Sticky_Leg_Plugin: " + Link_Name + "\n";

        // CREATE NECESSARY POINTERS
        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();
        Leg_Link_Ptr = Model_Ptr->GetLink(Link_Name);

        // INIT SERVICE AND UPDATE CALLBACK
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Sticky_Leg_Plugin::OnUpdate, this));
        Leg_Connect_Service = nh.advertiseService("/SAR_Internal/Sticky_Leg_" + std::to_string(Leg_Number), &Sticky_Leg_Plugin::Service_Callback, this);
    }

    void Sticky_Leg_Plugin::OnUpdate()
    {
        // IF STICKY ACTIVATED AND LINK NOT ALREADY ATTACHED
        // if (Sticky_Flag == true && Attached_Flag == false)
        // {
        // CHECK LIST OF COLLISIONS
        gazebo::physics::ContactManager *contactMgr = World_Ptr->Physics()->GetContactManager();
        const auto& contacts = contactMgr->GetContacts();

        for (const auto& contact : contacts)
        {
            // if (contact->collision1 && contact->collision2)
            // {
            //     std::cout << "Collision between: "
            //                 << contact->collision1->GetName() << " and "
            //                 << contact->collision2->GetName() << std::endl;
            // }

            if (Leg_Link_Ptr)
            {
                std::cout << "Leg Link: " << Leg_Link_Ptr->GetName() << std::endl;
            }
            else
            {
                std::cout << "Invalid Leg Link Ptr" << std::endl;
            }
            
            

        }

    

        /*
        for (unsigned int i = 0; i < contactMgr->GetContactCount(); i++)
        {

            // CHECK IF COLLISION LINK IS DESIRED LINK
            gazebo::physics::Contact *contact = contactMgr->GetContact(i);
            std::cout << contact->collision1->GetName() << "\t" << contact->collision2->GetName() << std::endl;
            std::cout << Leg_Link_Ptr->GetName() << std::endl;

            if (contact->collision1->GetLink() == Leg_Link_Ptr)
            {

                // // MARK CONNECTION AS ATTACHED
                // Surface_Link_Ptr = contact->collision2->GetLink();
                // Attached_Flag = true;

                // // CALCULATE JOINT LOCATION IN TERMS OF WORLD COORDINATES
                // contactPositionWorld = contact->positions[0];
                // contactPositionLocal = contact->collision2->GetLink()->WorldPose().Inverse().CoordPositionAdd(contactPositionWorld); // Position in surface frame of reference
                // contactPositionLocal -= ignition::math::Vector3d(0, 0, collision_radius);                                            // Offset for collision sphere radius
                // contactPositionWorldUpdated = contact->collision2->GetLink()->WorldPose().CoordPositionAdd(contactPositionLocal);    // Update global position
                // contactPose.Set(contactPositionWorldUpdated, ignition::math::Quaterniond::Identity);

                // // CREATE BALL JOINT BETWEEN LEG AND SURFACE AT DESIRED LOCATION
                // Contact_Joint_Ptr = Model_Ptr->CreateJoint(Joint_Name, "ball", Leg_Link_Ptr, Surface_Link_Ptr);
                // Contact_Joint_Ptr->SetAnchor(0, contactPose.Pos()); // Contact point offset by collision radius

                // printf("[Leg_%d]: Joint Created\t(%s->%s)\n", Leg_Number, Leg_Link_Ptr->GetName().c_str(), Surface_Link_Ptr->GetName().c_str());

                // // PUBLISH EACH TIME A LEG CONNECTS TO A SURFACE
                // switch(Leg_Number)
                // {
                //     case 1:
                //         Sticky_Leg_Connect_msg.Pad1_Contact = 1;
                //         break;
                //     case 2:
                //         Sticky_Leg_Connect_msg.Pad2_Contact = 1;
                //         break;
                //     case 3:
                //         Sticky_Leg_Connect_msg.Pad3_Contact = 1;
                //         break;
                //     case 4:
                //         Sticky_Leg_Connect_msg.Pad4_Contact = 1;
                //         break;
                // }

                // Sticky_Pad_Connect_Publisher.publish(Sticky_Leg_Connect_msg);
            }
        }*/
        // }
    }

    bool Sticky_Leg_Plugin::Service_Callback(sar_msgs::Activate_Sticky_Pads::Request &req, sar_msgs::Activate_Sticky_Pads::Response &res)
    {
        // TURN OFF STICKY BEHAVIOR
        if (req.Sticky_Flag == false)
        {
            Sticky_Flag = false;
            printf("[Leg_%d]: Sticky Disabled\n", Leg_Number);
        }

        // TURN OFF AND DETACH STICKY LEG
        if (req.Sticky_Flag == false && Attached_Flag == true)
        {
            // REMOVE CREATED JOINT
            Model_Ptr->RemoveJoint(Joint_Name);
            printf("[Leg_%d]: Joint Removed\t(%s->%s)\n", Leg_Number, Leg_Link_Ptr->GetName().c_str(), Surface_Link_Ptr->GetName().c_str());

            // RESET FLAGS AND POINTERS
            Attached_Flag = false;
            Contact_Joint_Ptr = NULL;
            Surface_Link_Ptr = NULL;
        }

        // TURN ON STICKY LEG
        if (req.Sticky_Flag == true)
        {
            Sticky_Flag = true;
            printf("[Leg_%d]: Sticky_Enabled\n", Leg_Number);
        }

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Sticky_Leg_Plugin);
}