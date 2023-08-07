
#include <Sticky_Leg_Plugin.h>

namespace gazebo
{
    void Sticky_Leg_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Sticky_Leg_Plugin\n";

        Joint_Name = _sdf->GetElement("JointName")->Get<std::string>();
        Link_Name = _sdf->GetElement("LinkName")->Get<std::string>();
        Leg_Number = _sdf->GetElement("LegNumber")->Get<int>();

        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();
        Leg_Link_Ptr = Model_Ptr->GetLink(Link_Name);

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Sticky_Leg_Plugin::OnUpdate, this));
        Leg_Connect_Service = nh.advertiseService("/SAR_Internal/Sticky_Leg_" + std::to_string(Leg_Number), &Sticky_Leg_Plugin::Service_Callback, this);

        printf("\n\n");
    }

    void Sticky_Leg_Plugin::OnUpdate()
    {
        // IF STICKY ACTIVATED AND LINK NOT ALREADY ATTACHED
        if (Sticky_Flag == true && Attached_Flag == false)
        {
            // CHECK LIST OF COLLISIONS
            gazebo::physics::ContactManager *contactMgr = World_Ptr->Physics()->GetContactManager();
            for (unsigned int i = 0; i < contactMgr->GetContactCount(); i++)
            {
                
                // CHECK IF COLLISION LINK IS DESIRED LINK
                gazebo::physics::Contact *contact = contactMgr->GetContact(i);
                if (contact->collision1->GetLink() == Leg_Link_Ptr)
                {

                    Surface_Link_Ptr = contact->collision2->GetLink();
                    Attached_Flag = true;

                    // CALCULATE JOINT LOCATION IN TERMS OF WORLD COORDINATES
                    contactPositionWorld = contact->positions[0];
                    contactPositionLocal = contact->collision2->GetLink()->WorldPose().Inverse().CoordPositionAdd(contactPositionWorld); // Position in surface frame of reference
                    contactPositionLocal -= ignition::math::Vector3d(0, 0, collision_radius); // Offset for collision sphere radius
                    contactPositionWorldUpdated = contact->collision2->GetLink()->WorldPose().CoordPositionAdd(contactPositionLocal); // Update global position
                    contactPose.Set(contactPositionWorldUpdated, ignition::math::Quaterniond::Identity);

                    // CREATE BALL JOINT BETWEEN LEG AND SURFACE
                    Contact_Joint_Ptr = Model_Ptr->CreateJoint(Joint_Name, "ball", Leg_Link_Ptr, Surface_Link_Ptr);
                    Contact_Joint_Ptr->SetAnchor(0, contactPose.Pos()); // Contact point offset by collision radius

                    // DISABLE FUTURE CONTACTS FOR LINK (ALLOWS FREE ROTATION)
                    Leg_Link_Ptr->SetCollideMode("none");
                    printf("[Leg_%d]: Joint Created\t(%s->%s)\n", Leg_Number, Leg_Link_Ptr->GetName().c_str(), Surface_Link_Ptr->GetName().c_str());
                
                }
            }
        }
    }

    bool Sticky_Leg_Plugin::Service_Callback(sar_msgs::Activate_Sticky_Pads::Request &req, sar_msgs::Activate_Sticky_Pads::Response &res)
    {
        if (req.Sticky_Flag == false)
        {
            // TURN OFF STICKY BEHAVIOR
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
            
            // REACTIVATE COLLISION IN LINK
            Leg_Link_Ptr->SetCollideMode("all");
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