#include "pub_example.h"


int main(int argc, char **argv)
{
    // INITIALIZE ROS NODE AND CREATE POINTER TO NODE
    ros::init(argc,argv,"Example_Node");
    ros::NodeHandle nh;

    // INITIALIZE CLASS AND SPIN ROS
    MyClass exampleClass = MyClass(&nh);
    ros::spin();
    return 1;
}