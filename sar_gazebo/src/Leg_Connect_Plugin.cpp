
#include <Leg_Connect_Plugin.h>

namespace gazebo
{

    void Leg_Connect_Plugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Leg_Connect_Plugin\n";
        
        Model_Ptr = _parent;
        World_Ptr = Model_Ptr->GetWorld();


        physics::ModelPtr Marker_Ptr = World_Ptr->ModelByName("Marker");
        physics::ModelPtr Surface_Ptr = World_Ptr->ModelByName("Surface");


        if (!Marker_Ptr) {
            gzerr << "Model 'Marker' not found after waiting!" << std::endl;
        }

        if (!Surface_Ptr) {
            gzerr << "Model 'Debug' not found after waiting!" << std::endl;
        }
        
        ignition::math::Pose3d contactPose;
        contactPose.Set(0,0,0.5,0,0,0);
        Marker_Ptr->SetWorldPose(contactPose);


        physics::LinkPtr Beam_Ptr = Model_Ptr->GetLink("beam_link");
        physics::JointPtr joint = World_Ptr->Physics()->CreateJoint("ball",Model_Ptr);
        joint->Attach(Surface_Ptr->GetLink("Surface_Link"),Beam_Ptr);
        joint->Load(Surface_Ptr->GetLink("Surface_Link"), Beam_Ptr, contactPose);
        joint->SetAnchor(0, contactPose.Pos());
        joint->Init();


        updateConnection = event::Events::ConnectWorldCreated(std::bind(&Leg_Connect_Plugin::Init, this));

        printf("\n\n");
    }

    void Leg_Connect_Plugin::Init()
    {

        


    }

    GZ_REGISTER_MODEL_PLUGIN(Leg_Connect_Plugin);
}