
/*
*  Name: gazebo_transport_to_ros_topic.cpp
*  Author: Joseph Coombe
*  Date: 11/22/2017
*  Edited: 11/27/2017
*  Description:
*   Subscribe to a Gazebo transport topic and publish to a ROS topic
*/

// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "crazyflie_rl/RLCmd.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <Eigen/Dense>

ros::Publisher impactForce_Publisher;
ros::Subscriber RLCmd_Subscriber;
ros::Subscriber globalState_Subscriber;

double ros_rate = 100; // Ros rate but not sure how it applies here
double _ceiling_ft_z = 0.0; // Max impact force in Z-direction [N]
double _ceiling_ft_x = 0.0; // Max impact force in X-direction [N]

Eigen::Vector3d _pos;   // Current position [m]
Eigen::Vector3d _vel;   // Current velocity [m]
Eigen::Vector4d _quat;  // Current attitude [rad] (quat form)
Eigen::Vector3d _omega; // Current angular velocity [rad/s]


void gazeboFT_Callback(const ConstWrenchStampedPtr &_msg)
{

  
  // Record max force experienced
  if (_msg->wrench().force().z() > _ceiling_ft_z){
    _ceiling_ft_z = _msg->wrench().force().z();
  }

  if (_msg->wrench().force().x() > _ceiling_ft_x){
    _ceiling_ft_x = _msg->wrench().force().x();
  }


  // std::cout << "Received msg: " << std::endl;
  // std::cout << _msg->DebugString() << std::endl;
  geometry_msgs::WrenchStamped msgWrenchedStamped;
  // try WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = _ceiling_ft_x;
  msgWrenchedStamped.wrench.force.y = _msg->wrench().force().y();
  msgWrenchedStamped.wrench.force.z = _ceiling_ft_z;
  msgWrenchedStamped.wrench.torque.x = _msg->wrench().torque().x();
  msgWrenchedStamped.wrench.torque.y = _msg->wrench().torque().y();
  msgWrenchedStamped.wrench.torque.z = _msg->wrench().torque().z();
  impactForce_Publisher.publish(msgWrenchedStamped);
 
}

void RLCmd_Callback(const crazyflie_rl::RLCmd::ConstPtr &msg)
{
  // When model is reset back to home position then reset max impact values
  if(msg->cmd_type == 0)
  {
    _ceiling_ft_z = 0.0;
    _ceiling_ft_x = 0.0;
  }

}

void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){

    // SIMPLIFY STATE VALUES FROM TOPIC
    // Follow msg names from message details - "rqt -s rqt_msg" 
    const geometry_msgs::Point position = msg->pose.pose.position; 
    const geometry_msgs::Vector3 velocity = msg->twist.twist.linear;
    const geometry_msgs::Quaternion quaternion = msg->pose.pose.orientation;
    const geometry_msgs::Vector3 omega = msg->twist.twist.angular;


    // SET STATE VALUES INTO GLOBAL STATE VARIABLES
    // _t = msg->header.stamp;
    _pos << position.x, position.y, position.z;
    _vel << velocity.x, velocity.y, velocity.z; 
    _quat << quaternion.w, quaternion.x, quaternion.y, quaternion.z, 
    _omega << omega.x, omega.y, omega.z;

}

int main(int argc, char **argv)
{
  // LOAD GAZEBO, CREATE GAZEBO NODE AND INIT
  ROS_INFO("Starting Gazebo node");
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // LOAD ROS, CREATE ROS NODE AND INIT
  ROS_INFO("Starting ROS node");
  ros::init(argc, argv, "gazebo_transport_to_ros_topic");
  ros::NodeHandle nh;
  ros::Rate loop_rate(ros_rate); 


  // DEFINE ROS AND GAZEBO TOPICS
  std::string gazebo_transport_topic_to_sub= "/gazebo/default/ceiling_plane/joint_01/force_torque/wrench";
  std::string ros_topic_to_pub="/ceiling_force_sensor";
  
  // INIT ROS PUBLISHERS/SUBSCRIBERS
  impactForce_Publisher = nh.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub, 10);
  globalState_Subscriber = nh.subscribe("/global_state",1,global_stateCallback,ros::TransportHints().tcpNoDelay());
  RLCmd_Subscriber = nh.subscribe("/rl_ctrl",50,RLCmd_Callback);
  ROS_INFO("ROS Subscribers/Publishers Started");
  


  // LISTEN TO GAZEBO FORCE_TORQUE SENSOR TOPIOC
  gazebo::transport::SubscriberPtr sub = node->Subscribe(gazebo_transport_topic_to_sub, gazeboFT_Callback);
  ROS_INFO("Gazebo Subscriber Started");
  
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep(); // I don't think this is even doing anything since everything is inside the torqueCb
    
  }
  gazebo::shutdown();
  return 0;
}