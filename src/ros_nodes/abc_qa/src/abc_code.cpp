#include <ros/ros.h>
#include <std_msgs/Int32.h>

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