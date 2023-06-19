#include <iostream>

// ROS/GAZEBO IMPORTS
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

// CUSTOM IMPORTS
#include "sar_msgs/PadConnect.h"
#include "sar_msgs/activateSticky.h"


namespace gazebo {

    class GazeboStickyFoot: public ModelPlugin
    {
        public:
            
        protected:
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            void ContactCallback(ConstContactsPtr &msg);
            bool activateSticky(sar_msgs::activateSticky::Request &req, sar_msgs::activateSticky::Response &res);

        private:
            physics::WorldPtr world_;
            physics::ModelPtr model_;
            physics::LinkPtr padLink_ptr;
            physics::LinkPtr contactLink_ptr;
            physics::JointPtr joint_ptr;
            physics::PhysicsEnginePtr physics_engine_;
            physics::ContactManager *contact_manager_;

            transport::NodePtr node_handle_;
            transport::SubscriberPtr contact_sub_;

    
            ros::NodeHandle nh;
            ros::Publisher PadConnect_Publisher = nh.advertise<sar_msgs::PadConnect>("/ENV/Pad_Connections", 5);
            ros::ServiceServer stickyService;

            std::string namespace_;
            std::string padName;
            std::string jointName;
            std::string contact_pub_topic;
            std::string serviceName;

            bool sticky_flag;
            int PAD_NUMBER;

            sar_msgs::PadConnect PadConnect_msg;
    };

}