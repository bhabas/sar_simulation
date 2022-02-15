#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>




#include <ros/ros.h>
#include "crazyflie_msgs/PadConnect.h"
#include "crazyflie_msgs/RLCmd.h"
#include "crazyflie_msgs/activateSticky.h"


namespace gazebo {

class GazeboStickyFoot: public ModelPlugin
{
    public: 

    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        //void OnUpdate(const common::UpdateInfo&  /*_info*/);
        void ContactCallback(ConstContactsPtr &msg);
        bool activateSticky(crazyflie_msgs::activateSticky::Request &req, crazyflie_msgs::activateSticky::Response &res);
        
        

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

        std::string namespace_;
        std::string padName;
        std::string jointName;
        std::string contact_pub_topic;
        std::string serviceName;

        event::ConnectionPtr updateConnection_;

        ros::NodeHandle n;
        ros::Publisher PadConnect_Publisher = n.advertise<crazyflie_msgs::PadConnect>("/pad_connections", 5);
        ros::ServiceServer stickyService;
        
        
        
        bool sticky_;
        double vz_max_;
        int pad_number_;


};

}