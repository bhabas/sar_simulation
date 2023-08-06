
#include <Leg_Connect_Plugin.h>

namespace gazebo
{
    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";

        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();

        Marker_Ptr = World_Ptr->ModelByName("Marker");
        Surface_Ptr = World_Ptr->ModelByName("Surface");

        if (!Marker_Ptr)
        {
            gzerr << "Model 'Marker' not found after waiting!" << std::endl;
        }

        if (!Surface_Ptr)
        {
            gzerr << "Model 'Debug' not found after waiting!" << std::endl;
        }

        
        contactPose.Set(0, 0, 0, 0, 0, 0);
        Marker_Ptr->SetWorldPose(contactPose);

        // physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
        // physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
        // joint->Attach(NULL, Beam_Ptr);
        // joint->Load(NULL, Beam_Ptr, contactPose);
        // joint->SetAnchor(0, contactPose.Pos());
        // joint->Init();

        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Leg_Connect_Plugin::OnUpdate, this));

        printf("\n\n");
    }

    void Leg_Connect_Plugin::OnUpdate()
    {

        gazebo::physics::ContactManager *contactMgr = World_Ptr->Physics()->GetContactManager();
        unsigned int collisionCount = contactMgr->GetContactCount();

        std::cout << collisionCount << std::endl;

        
        

        for (unsigned int i = 0; i < collisionCount; i++)
        {
            gazebo::physics::Contact *contact = contactMgr->GetContact(i);

            std::cout << contact->collision1->GetModel()->GetName() << std::endl;
            std::cout << contact->collision2->GetModel()->GetName() << std::endl;

            // Check the models involved in the collision
            if (contact->collision1->GetModel() == Model_Ptr || contact->collision2->GetModel() == Model_Ptr)
            {

                if (OnceFlag == false)
                {
                    OnceFlag = true;

                    std::cout << "Starting Joint" << std::endl;
                    
                    physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
                    physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
                    joint->Attach(Surface_Ptr->GetLink("Surface_Link"), Beam_Ptr);
                    joint->Load(Surface_Ptr->GetLink("Surface_Link"), Beam_Ptr, contactPose);
                    joint->SetAnchor(0, contactPose.Pos());
                    joint->Init();

                    std::cout << "Finish Joint" << std::endl;
                }


                // if (OnceFlag == false)
                // {
                //     OnceFlag = true;

                    

                //     // physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
                //     // physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
                //     // joint->Attach(Surface_Ptr->GetLink("Surface_Link"), Beam_Ptr);
                //     // joint->Load(Surface_Ptr->GetLink("Surface_Link"), Beam_Ptr, contactPose);
                //     // joint->SetAnchor(0, contactPose.Pos());
                //     // joint->Init();

                //     physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
                //     physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
                //     joint->Attach(NULL, Beam_Ptr);
                //     joint->Load(NULL, Beam_Ptr, contactPose);
                //     joint->SetAnchor(0, contactPose.Pos());
                //     joint->Init();

                //     std::cout << "Finish Joint" << std::endl;
                // }
                
                

            }
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}