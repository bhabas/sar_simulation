
#include <Leg_Connect_Plugin.h>

namespace gazebo
{

    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";
        
        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();

        Collision_Name = "Leg_1_Collision";
      
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Leg_Connect_Plugin::OnUpdate, this));

        OnceFlag = false;

        printf("\n\n");
    }

    void Leg_Connect_Plugin::OnUpdate()
    {

        
        physics::ContactManager* contactManager = World_Ptr->Physics()->GetContactManager();
        unsigned int collisionCount = contactManager->GetContactCount();
        
        std::cout << collisionCount << std::endl;

        for (unsigned int i = 0; i < collisionCount; ++i)
        {
            physics::Contact* contact = contactManager->GetContact(i);


            std::cout << contact->collision1->GetName() << std::endl;
            std::cout << contact->collision2->GetName() << std::endl;

          if (contact->collision1->GetName() == Collision_Name || contact->collision2->GetName() == Collision_Name)
          {

            std::cout << contact->collision1->GetName() << std::endl;
            std::cout << contact->collision2->GetName() << std::endl;

            if (OnceFlag == false)
            {
                OnceFlag = true;
                Joint_Ptr = Model_Ptr->CreateJoint("Test_Joint","ball",contact->collision1->GetLink(),contact->collision2->GetLink());
                std::cout << "Joint Created" << std::endl;
            }
            
            

            // // Contact detected. Create a ball joint
            // Joint_Ptr = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
            // Joint_Ptr->Attach(contact->collision1->GetLink(), contact->collision2->GetLink());

            // ignition::math::Pose3d contact_pose;
            // Joint_Ptr->Load(contact->collision1->GetLink(), contact->collision2->GetLink(), ignition::math::Pose3d());
            // Joint_Ptr->Init();

            // Optionally, break the connection to stop checking for contacts
            // updateConnection.reset();
          }
        }
        
        

    }


    bool Leg_Connect_Plugin::Service_Callback(sar_msgs::Cam_Settings::Request &req, sar_msgs::Cam_Settings::Response &res)
    {

        return true;
    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}