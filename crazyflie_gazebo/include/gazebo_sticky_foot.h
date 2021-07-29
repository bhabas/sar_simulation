#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include "common.h"     // provides getSdfParam, FirstOrderFilter ...

#include "CommandMotorSpeed.pb.h"


#include <ros/ros.h>
#include "crazyflie_msgs/PadConnect.h"
//#include "gazebo_motor_model.h"     // provides CommandMotorSpeedPtr

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

namespace gazebo {

class GazeboStickyFoot: public ModelPlugin
{
    public: 

    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        //void OnUpdate(const common::UpdateInfo&  /*_info*/);
        void ContactCallback(ConstContactsPtr &msg);
        void StickyEnableCallback(CommandMotorSpeedPtr &rot_velocities);

    private:
        physics::ModelPtr model_;
        physics::LinkPtr link_;
        physics::LinkPtr link2_;
        physics::WorldPtr world_;
        physics::PhysicsEnginePtr physics_engine_;
        physics::ContactManager *contact_manager_;
        physics::JointPtr joint_;

        transport::NodePtr node_handle_;
        //transport::NodePtr contact_node_;
        transport::SubscriberPtr sticky_enable_sub_;
        transport::SubscriberPtr contact_sub_;

        std::string namespace_;
        std::string link_name_;
        std::string sticky_enable_sub_topic_;
        std::string contact_pub_topic;
        std::string joint_name_;

        event::ConnectionPtr updateConnection_;

        ros::NodeHandle n;
        ros::Publisher PadConnect_Publisher = n.advertise<crazyflie_msgs::PadConnect>("/pad_connections", 5);

        
        bool sticky_;
        double vz_max_;
        int pad_number_;


};

}