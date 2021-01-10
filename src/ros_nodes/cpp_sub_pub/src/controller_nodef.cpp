#include "ros/ros.h"
#include "cpp_sub_pub/my_msg.h"
#include <std_msgs/Int32.h>



int main(int argc, char **argv)
{
  //Initialize and start ros node
  ros::init(argc,argv,"MS_pub");
  ros::NodeHandle nh; 

  //Define the Publisher
  ros::Publisher pub = nh.advertise<cpp_sub_pub::my_msg>("MS_topic",1000);
  ros::Rate rate(200);

  //Counter variable
  float i = 0;

  while(ros::ok()){
    i+=1;
    // Define msg object
    cpp_sub_pub::my_msg msg;
    msg.motorspeeds = {1+i,2+i,3+i,4+i};

    // Publish msg and sleep
    pub.publish(msg);
    rate.sleep();
 }
}