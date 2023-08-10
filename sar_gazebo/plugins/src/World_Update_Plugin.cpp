#include <World_Update_Plugin.h>

namespace gazebo
{
    void World_Update_Plugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading World_Update_Plugin\n";

        // LOAD MODEL AND LINK POINTERS
        World_Ptr = _world;
        
        
        ros::param::get("/SIM_SETTINGS/Sim_Speed",Sim_Speed);
        ros::param::get("/SIM_SETTINGS/Gravity",Gravity_Flag);

        
        Update_Gravity();
        Update_SimSpeed();

        printf("\n\n");
    }

    void World_Update_Plugin::Update_Gravity()
    {
        if (Gravity_Flag == true)
        {
           World_Ptr->SetGravity(ignition::math::Vector3d(0, 0, -9.8066));
        }
        else
        {
            World_Ptr->SetGravity(ignition::math::Vector3d(0, 0, 0));
        }
        
    }

    void World_Update_Plugin::Update_SimSpeed()
    {
        World_Ptr->Physics()->SetRealTimeUpdateRate(Sim_Speed/Step_Size);
    }



    GZ_REGISTER_WORLD_PLUGIN(World_Update_Plugin);
}