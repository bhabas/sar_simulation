#include <iostream>
#include "OF_Sensor_plugin.h"


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void OF_SensorPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading OF_SensorPlugin\n";
        model_ = parent;

        linkName = _sdf->GetElement("bodyName")->Get<std::string>();
        gzmsg << "\t Link Name:\t" << linkName << std::endl;
        link_ptr = model_->GetLink(linkName);


        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OF_SensorPlugin::OnUpdate, this));

        // ROS_WARN("loaded ModelPush Plugin with parent...%s",model_->GetName().c_str());

        std::cout << "\n\n";
    }

    void OF_SensorPlugin::OnUpdate()
    {
        // DEFINE
        Vx_rel = link_ptr->RelativeLinearVel().X();
        Vy_rel = link_ptr->RelativeLinearVel().Y();
        Vz_rel = link_ptr->RelativeLinearVel().Z();
        d_ceil = h_ceiling - link_ptr->WorldPose().Z();

        Tau = d_ceil/Vz_rel;
        RREV = Vz_rel/d_ceil;
        OFx = -Vy_rel/d_ceil;
        OFy = -Vx_rel/d_ceil;

        printf("Tau: %.3f \t RREV: %.3f \t OFx: %.3f \t OFy: %.3f \t d_ceil: %.3f\n",Tau,RREV,OFx,OFy,d_ceil);

        model_->SetLinearVel(ignition::math::Vector3d(0.5, 0.5, 2.5));

    }



    // void GazeboMotorPlugin::MotorSpeedCallback(const crazyflie_msgs::MS::ConstPtr &msg)
    // {

    // }

    GZ_REGISTER_MODEL_PLUGIN(OF_SensorPlugin);
}