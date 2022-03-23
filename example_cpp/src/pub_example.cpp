#include "pub_example.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ros/ros.h>


int main(int argc, char **argv)
{
    
    // INITIALIZE ROS NODE AND CREATE POINTER TO NODE
    ros::init(argc,argv,"Camera_Node");
    ros::NodeHandle nh;
    
    // INITIALIZE CLASS AND SPIN ROS
    MyClass exampleClass = MyClass(&nh);
    ros::spin();
    return 1; //return one means proper system exit

}