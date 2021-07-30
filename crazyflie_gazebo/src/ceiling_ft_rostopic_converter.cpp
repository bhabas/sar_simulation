
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
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "nav_msgs/Odometry.h"


#include <iostream>
#include <Eigen/Dense>


ros::Publisher impactForce_Publisher;
ros::Subscriber RLdata_Subscriber;
ros::Subscriber globalState_Subscriber;

double _ceiling_ft_x = 0.0; // Max impact force in X-direction [N]
double _ceiling_ft_y = 0.0; // Max impact force in Y-direction [N]
double _ceiling_ft_z = 0.0; // Max impact force in Z-direction [N]
bool _impact_flag = false;


geometry_msgs::Point _pos;        // Current position [m]
geometry_msgs::Vector3 _vel;      // Current velocity [m]
geometry_msgs::Quaternion _quat;  // Current attitude [rad] (quat form)
geometry_msgs::Vector3 _omega;    // Current angular velocity [rad/s]


geometry_msgs::Vector3 _vel_prev;      // Current velocity [m]

ros::Time _t_impact;                      // Impact time [s]
geometry_msgs::Point _pos_impact;         // Impact position [m]
geometry_msgs::Vector3 _vel_impact;       // Impact velocity [m]
geometry_msgs::Quaternion _quat_impact;   // Impact attitude [rad] (quat form)
geometry_msgs::Vector3 _omega_impact;     // Impact angular velocity [rad/s]



const int arr_len = 5;
geometry_msgs::Point _pos_arr [arr_len];
geometry_msgs::Vector3 _vel_arr [arr_len];
geometry_msgs::Quaternion _quat_arr [arr_len];
geometry_msgs::Vector3 _omega_arr [arr_len];


void gazeboFT_Callback(const ConstWrenchStampedPtr &_msg)
{
  // Record max force experienced
  if (_msg->wrench().force().x() > _ceiling_ft_x){
    _ceiling_ft_x = _msg->wrench().force().x();
  }
  if (_msg->wrench().force().y() > _ceiling_ft_y){
    _ceiling_ft_y = _msg->wrench().force().y();
  }
  if (_msg->wrench().force().z() > _ceiling_ft_z){
    _ceiling_ft_z = _msg->wrench().force().z();
  }



  if (_ceiling_ft_z >= 0.1 && _impact_flag == false){ 
    // LOCK IN STATE DATA WHEN IMPACT DETECTED
    _impact_flag = true;


    // RECORD IMPACT STATE DATA FROM [2] DATAPOINTS BEHIND WHEN IMPACT FLAGGED
    _t_impact = ros::Time::now();
    _pos_impact = _pos_arr[2];
    _vel_impact = _vel_arr[2];
    _quat_impact = _quat_arr[2];
    _omega_impact = _omega_arr[2];

  }

  //PUBLISH THAT IMPACT OCCURED
  crazyflie_msgs::ImpactData impact_msg;

  impact_msg.impact_flag = _impact_flag;
  impact_msg.Header.stamp = _t_impact;

  // WRITE CURRENT MAX IMPACT FORCES TO MSG
  impact_msg.Force_impact.x = _ceiling_ft_x;
  impact_msg.Force_impact.y = _ceiling_ft_y;
  impact_msg.Force_impact.z = _ceiling_ft_z;

  // WRITE LAGGING IMPACT POSE TO MSG
  impact_msg.Pose_impact.position.x = _pos_impact.x;
  impact_msg.Pose_impact.position.y = _pos_impact.y;
  impact_msg.Pose_impact.position.z = _pos_impact.z;

  impact_msg.Pose_impact.orientation.x = _quat_impact.x;
  impact_msg.Pose_impact.orientation.y = _quat_impact.y;
  impact_msg.Pose_impact.orientation.z = _quat_impact.z;
  impact_msg.Pose_impact.orientation.w = _quat_impact.w;

  // WRITE LAGGING IMPACT TWIST TO MSG
  impact_msg.Twist_impact.linear.x = _vel_impact.x;
  impact_msg.Twist_impact.linear.y = _vel_impact.y;
  impact_msg.Twist_impact.linear.z = _vel_impact.z;
  
  impact_msg.Twist_impact.angular.x = _omega_impact.x;
  impact_msg.Twist_impact.angular.y = _omega_impact.y;
  impact_msg.Twist_impact.angular.z = _omega_impact.z;

  impactForce_Publisher.publish(impact_msg);
  
}

void RLdata_Callback(const crazyflie_msgs::RLData::ConstPtr &msg)
{
  // WHEN RUN COMPLETED PUBLISH IMPACT DATA
  if(msg->reset_flag == true)
  {

    // RESET IMPACT FLAG AND VALUES
    _impact_flag = false;
    _ceiling_ft_x = 0.0;
    _ceiling_ft_y = 0.0;
    _ceiling_ft_z = 0.0;

    // RESET IMPACT VALUES WHENEVER RESET IS CALLED
    std::tie(_pos_impact.x,_pos_impact.y,_pos_impact.z) = std::make_tuple(0.0,0.0,0.0);
    std::tie(_vel_impact.x,_vel_impact.y,_vel_impact.z) = std::make_tuple(0.0,0.0,0.0);
    std::tie(_omega_impact.x,_omega_impact.y,_omega_impact.z) = std::make_tuple(0.0,0.0,0.0);
    std::tie(_quat_impact.x,_quat_impact.y,_quat_impact.z,_quat_impact.w) = std::make_tuple(0.0,0.0,0.0,1.0);


  }

}

void global_stateCallback(const nav_msgs::Odometry::ConstPtr &msg){

    // SET STATE VALUES INTO GLOBAL STATE VARIABLES
    _pos = msg->pose.pose.position; 
    _quat = msg->pose.pose.orientation;
    _vel = msg->twist.twist.linear;
    _omega = msg->twist.twist.angular;


    // SHIFT ALL ARRAY VALUES OVER BY ONE
    // Totally not the 'correct' way but it works for what I need right now

    _pos_arr[4] = _pos_arr[3];
    _pos_arr[3] = _pos_arr[2];
    _pos_arr[2] = _pos_arr[1];
    _pos_arr[1] = _pos_arr[0];
    _pos_arr[0] = msg->pose.pose.position;

    _vel_arr[4] = _vel_arr[3];
    _vel_arr[3] = _vel_arr[2];
    _vel_arr[2] = _vel_arr[1];
    _vel_arr[1] = _vel_arr[0];
    _vel_arr[0] = msg->twist.twist.linear;

    _omega_arr[4] = _omega_arr[3];
    _omega_arr[3] = _omega_arr[2];
    _omega_arr[2] = _omega_arr[1];
    _omega_arr[1] = _omega_arr[0];
    _omega_arr[0] = msg->twist.twist.angular;

    _quat_arr[4] = _quat_arr[3];
    _quat_arr[3] = _quat_arr[2];
    _quat_arr[2] = _quat_arr[1];
    _quat_arr[1] = _quat_arr[0];
    _quat_arr[0] = msg->pose.pose.orientation;

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


  // DEFINE ROS AND GAZEBO TOPICS
  std::string gazebo_topic_to_sub= "/gazebo/default/ceiling_plane/joint_01/force_torque/wrench";
  std::string ros_topic_to_pub="/ceiling_force_sensor";
  
  // INIT ROS PUBLISHERS/SUBSCRIBERS
  impactForce_Publisher = nh.advertise<crazyflie_msgs::ImpactData>(ros_topic_to_pub, 1);
  globalState_Subscriber = nh.subscribe("/global_state",1,global_stateCallback,ros::TransportHints().tcpNoDelay());
  RLdata_Subscriber = nh.subscribe("/rl_data",5,RLdata_Callback);
  ROS_INFO("ROS Subscribers/Publishers Started");
  


  // LISTEN TO GAZEBO FORCE_TORQUE SENSOR TOPIOC
  gazebo::transport::SubscriberPtr sub = node->Subscribe(gazebo_topic_to_sub, gazeboFT_Callback);
  ROS_INFO("Gazebo Subscriber Started");
  
  ros::spin();
  gazebo::shutdown();
  return 0;
}