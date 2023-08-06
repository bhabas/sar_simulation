
#include <Leg_Connect_Plugin.h>

namespace gazebo
{
    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";

        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();

        Joint_Name = _sdf->GetElement("JointName")->Get<std::string>();
        Link_Name = _sdf->GetElement("LinkName")->Get<std::string>();


        Leg_Ptr = Model_Ptr->GetLink(Link_Name);


        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Leg_Connect_Plugin::OnUpdate, this));

        printf("\n\n");
    }

    void Leg_Connect_Plugin::OnUpdate()
    {

        gazebo::physics::ContactManager *contactMgr = World_Ptr->Physics()->GetContactManager();
        unsigned int collisionCount = contactMgr->GetContactCount();

        for (unsigned int i = 0; i < collisionCount; i++)
        {
            gazebo::physics::Contact *contact = contactMgr->GetContact(i);

            std::cout << "Collision_1: " + contact->collision1->GetModel()->GetName() << std::endl;
            std::cout << "Collision_2: " + contact->collision2->GetModel()->GetName() << std::endl;

            // Check the models involved in the collision
            if (contact->collision1->GetModel() == Model_Ptr)
            {
                if (!OnceFlag)
                {
                    Surface_Model_Ptr = contact->collision2->GetModel();
                    Surface_Link_Ptr = Surface_Model_Ptr->GetLink("Surface_Link");

                    ignition::math::Vector3d contactPositionWorld = contact->positions[0];
                    ignition::math::Vector3d contactPositionSurfaceLocal = contact->collision2->GetLink()->WorldPose().Inverse().CoordPositionAdd(contactPositionWorld);
                    contactPositionSurfaceLocal -= ignition::math::Vector3d(0,0,collision_radius);
                    ignition::math::Vector3d contactPositionWorldUpdated = contact->collision2->GetLink()->WorldPose().CoordPositionAdd(contactPositionSurfaceLocal);
                    ignition::math::Pose3d contactPose(contactPositionWorldUpdated, ignition::math::Quaterniond::Identity);


                    OnceFlag = true;
                    std::cout << "Starting Joint" << std::endl;
                    std::cout << contact->positions[0] << std::endl;

                    std::cout << contactPositionWorld << std::endl;
                    std::cout << contactPositionSurfaceLocal << std::endl;
                    std::cout << contactPositionWorldUpdated << std::endl;


                    
                    
                    Joint_Ptr = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
                    Joint_Ptr->Attach(Leg_Ptr,Surface_Link_Ptr);
                    Joint_Ptr->SetName(Joint_Name);
                    Joint_Ptr->Load(Leg_Ptr,Surface_Link_Ptr, contactPose);
                    Joint_Ptr->SetAnchor(0, contactPose.Pos());
                    Joint_Ptr->Init();

                    Leg_Ptr->SetCollideMode("none");



                    std::cout << "Finish Joint" << std::endl;
                }

            }
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}