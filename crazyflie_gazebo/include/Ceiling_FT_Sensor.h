#include <ros/ros.h>
#include "example_msgs/CustomMessage.h"

class MyClass
{
    public:

        MyClass(ros::NodeHandle *nh)
        {
            pub = nh->advertise<example_msgs::CustomMessage>("/MyPub_cpp",1);

            while(true)
            {
                example_msgs::CustomMessage new_msg;
                new_msg.custom_msg = "Hello c++";
                new_msg.custom_vector.x = 5;

                pub.publish(new_msg);
            }
        }

        ros::Publisher pub;

};