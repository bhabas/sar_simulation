
#include <Leg_Connect_Plugin.h>

namespace gazebo
{

    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";
        
        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();

        ignition::math::Vector3d contactPosition;
        contactPosition.Set(1,0,0.1);

        physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
        physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball",Model_Ptr);
        joint->Attach(NULL,Beam_Ptr);
        joint->Load(NULL, Beam_Ptr, ignition::math::Pose3d(contactPosition, ignition::math::Quaterniond::Identity));
        joint->SetAnchor(0, contactPosition);
        joint->Init();

        // updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Leg_Connect_Plugin::OnUpdate, this));

        printf("\n\n");
    }

    void Leg_Connect_Plugin::OnUpdate()
    {


    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}