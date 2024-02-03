#include <Step_World_Plugin.h>


namespace gazebo
{
    void Step_World_Plugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading Step_World_Plugin\n";
        World_Ptr = _parent;


        printf("\n\n");
    }

    bool Step_World_Plugin::Step_World(sar_msgs::World_Step::Request &req, sar_msgs::World_Step::Response &res)
    {
        gzmsg << "Step_World (n_steps: " << req.n_steps << ")\n";
        int n_steps = req.n_steps;

        World_Ptr->SetPaused(true);
        World_Ptr->Step(n_steps);

        return true;
    }



    GZ_REGISTER_WORLD_PLUGIN(Step_World_Plugin);
}