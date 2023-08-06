
#include <Leg_Connect_Plugin.h>

namespace gazebo
{
    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";

        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();


        


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
                    Surface_Ptr = contact->collision2->GetModel();

                    ignition::math::Vector3d contactPositionWorld = contact->positions[0];
                    ignition::math::Vector3d contactPositionSurfaceLocal = Surface_Ptr->GetLink("Surface_Link")->WorldPose().Inverse().CoordPositionAdd(contactPositionWorld);
                    contactPositionSurfaceLocal += ignition::math::Vector3d(0,0,-0.01);
                    ignition::math::Pose3d contactPose(contactPositionSurfaceLocal, ignition::math::Quaterniond::Identity);


                    OnceFlag = true;
                    std::cout << "Starting Joint" << std::endl;
                    std::cout << contact->positions[0] << std::endl;

                    std::cout << contactPositionWorld << std::endl;
                    std::cout << contactPositionSurfaceLocal << std::endl;

                    
                    physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
                    physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball", Model_Ptr);
                    joint->Attach(Beam_Ptr,Surface_Ptr->GetLink("Surface_Link"));
                    joint->Load(Beam_Ptr,Surface_Ptr->GetLink("Surface_Link"), contactPose);
                    joint->SetAnchor(0, contactPositionSurfaceLocal);
                    joint->Init();

                    Beam_Ptr->SetCollideMode("none");



                    std::cout << "Finish Joint" << std::endl;
                }

            }
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}