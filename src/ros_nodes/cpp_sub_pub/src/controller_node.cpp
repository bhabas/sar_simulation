#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <gazebo_communication_pkg/GlobalState.h>

#include "controller_node.h"

#include "cpp_sub_pub/my_msg.h"

// Reference: https://roboticsbackend.com/oop-with-ros-in-cpp/



void Controller::callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg){
    const ros::Time a = msg->header.stamp;
    const geometry_msgs::Point b = msg->global_pose.position;
    alpha = b.z;
    // std::cout << b.z << std::endl;

}

void Controller::controlThread(){

  float motorspeed[4];
  float i=0;
  ros::Rate rate(200);
  while(ros::ok()){
    i+=1;
    std::cout << alpha << std::endl;
    cpp_sub_pub::my_msg msg;
    msg.motorspeeds = {1+i,2+i,3+i,4+i};
    pub.publish(msg);
    rate.sleep();


  }
}



int main(int argc, char **argv)
{
    
    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);
    ros::spin();

    // while(ros::ok()){

    //   // std::cout << controller.alpha << std::endl;
    //   int c = 0;
    // }
}