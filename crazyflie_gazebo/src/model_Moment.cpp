#include <iostream>
#include <model_Moment.h>


namespace gazebo
{

    // This gets called when model is loaded and pulls values from sdf file
    void ModelMoment::Load(physics::ModelPtr parent, sdf::ElementPtr _sdf)
    {
        gzmsg << "Loading GazeboMomentPlugin\n";
        model_ = parent;
        
        // RUN FUNCTION EACH TIME SIMULATION UPDATES
        updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelMoment::OnUpdate, this));
        printf("\n\n");

    }

    void ModelMoment::OnUpdate()
    {
       
    }

    // void GazeboMotorPlugin::updateTorque()
    // {
    //     // APPLY ROTOR TORQUE TO MAIN BODY
    //     torque = torque_coeff*thrust;
    //     ignition::math::Vector3d torque_vec(0, 0, -turning_direction * torque); // Torque is opposite direction of rotation

    //     physics::Link_V parent_links = link_ptr->GetParentJointsLinks(); // Get <vector> of parent links
    //     ignition::math::Pose3d pose_difference = link_ptr->WorldCoGPose() - parent_links.at(0)->WorldCoGPose(); // Find rotor pos relative to body
    //     ignition::math::Vector3d torque_parent_frame = pose_difference.Rot().RotateVector(torque_vec); // Rotate torque vector to match body orientation
    //     parent_links.at(0)->AddRelativeTorque(torque_parent_frame); // Apply torque vector to body
    // }

    void ModelMoment::RLCmd_Callback(const crazyflie_msgs::RLCmd::ConstPtr &msg)
    {
        // rotorPWM = msg->MotorPWM[motor_number-1];
    }

    GZ_REGISTER_MODEL_PLUGIN(ModelMoment);
}