#include <ros/ros.h>
#include <std_msgs/Int32.h>

// working example for c++ publisher
// https://www.youtube.com/watch?v=s7kluSveBuE&list=PLK0b4e05LnzbrLrLhOhSvLdaQZOsJl59N&index=68&t=598s
int main(int argc, char** argv)
{
    //Initialize and start ros node
    ros::init(argc, argv,"abc");
    ros::NodeHandle nh;

    //Define the Publisher
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("abc_topic",1000);

    //Define and create some messages
    std_msgs::Int32 abc;
    abc.data = 1;
    ros::Rate rate(200);

    while(ros::ok()){
        pub.publish(abc);
        abc.data++;
        rate.sleep();
        
    }


}