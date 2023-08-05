
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


        printf("\n\n");
    }

    void Leg_Connect_Plugin::OnUpdate()
    {
        physics::ContactManager* contactManager = World_Ptr->Physics()->GetContactManager();
        unsigned int collisionCount = contactManager->GetContactCount();

        for (unsigned int i = 0; i < collisionCount; ++i)
        {
          physics::Contact* contact = contactManager->GetContact(i);

          if (contact->collision1->GetName() == Collision_Name || contact->collision2->GetName() == Collision_Name)
          {
            // Contact detected. Create a ball joint
            Joint_Ptr = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
            Joint_Ptr->Attach(contact->collision1->GetLink(), contact->collision2->GetLink());

            ignition::math::Pose3d contact_pose;
            Joint_Ptr->Load(contact->collision1->GetLink(), contact->collision2->GetLink(), ignition::math::Pose3d());
            Joint_Ptr->Init();

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