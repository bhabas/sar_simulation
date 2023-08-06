
#include <Sticky_Leg_Plugin.h>

namespace gazebo
{
    void Sticky_Leg_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Sticky_Leg_Plugin\n";

        Joint_Name = _sdf->GetElement("JointName")->Get<std::string>();
        Link_Name = _sdf->GetElement("LinkName")->Get<std::string>();
        Leg_Number = _sdf->GetElement("LegNumber")->Get<std::string>();


        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();
        Leg_Link_Ptr = Model_Ptr->GetLink(Link_Name);

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Sticky_Leg_Plugin::OnUpdate, this));
        Leg_Connect_Service = nh.advertiseService("/SAR_Internal/Sticky_Leg_" + Leg_Number, &Sticky_Leg_Plugin::Service_Callback, this);


        printf("\n\n");
    }

    void Sticky_Leg_Plugin::OnUpdate()
    {

        gazebo::physics::ContactManager *contactMgr = World_Ptr->Physics()->GetContactManager();
        unsigned int collisionCount = contactMgr->GetContactCount();

        for (unsigned int i = 0; i < collisionCount; i++)
        {
            gazebo::physics::Contact *contact = contactMgr->GetContact(i);

            std::cout << "Collision_1: " + contact->collision1->GetModel()->GetName() << std::endl;
            std::cout << "Collision_2: " + contact->collision2->GetModel()->GetName() << std::endl;

            // Check the models involved in the collision
            if (contact->collision1->GetLink() == Leg_Link_Ptr)
            {
                if (!Sticky_Flag)
                {
                    Sticky_Flag = true;
                    Surface_Link_Ptr = contact->collision2->GetLink();

                    std::cout << contact->collision2->GetModel()->GetName() << std::endl;
                    std::cout << contact->collision2->GetLink()->GetName() << std::endl;


                    contactPositionWorld = contact->positions[0];
                    contactPositionLocal = contact->collision2->GetLink()->WorldPose().Inverse().CoordPositionAdd(contactPositionWorld);
                    contactPositionLocal -= ignition::math::Vector3d(0,0,collision_radius); // Offset for collision sphere radius
                    contactPose.Set(contactPositionLocal, ignition::math::Quaterniond::Identity);

                    std::cout << "Starting Joint" << std::endl;
                    std::cout << contactPositionWorld << std::endl;
                    std::cout << contactPositionLocal << std::endl;

                    Contact_Joint_Ptr = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
                    Contact_Joint_Ptr->Attach(Leg_Link_Ptr,Surface_Link_Ptr);
                    Contact_Joint_Ptr->SetName(Joint_Name);
                    Contact_Joint_Ptr->Load(Leg_Link_Ptr,Surface_Link_Ptr, contactPose);
                    Contact_Joint_Ptr->SetAnchor(0, contactPose.Pos());
                    Contact_Joint_Ptr->Init();

                    Leg_Link_Ptr->SetCollideMode("none");
                    std::cout << "Finish Joint" << std::endl;
                }

            }
        }
    }

    bool Sticky_Leg_Plugin::Service_Callback(sar_msgs::Activate_Sticky_Pads::Request &req, sar_msgs::Activate_Sticky_Pads::Response &res)
    {
        printf("dfdfasd\n\n\n\n");

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Sticky_Leg_Plugin);
}