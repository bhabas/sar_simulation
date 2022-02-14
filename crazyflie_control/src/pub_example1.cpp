#include "pub_example.h"
// #include <iostream>

int main(int argc, char **argv)
{
    std::cout << "Hello World" << std::endl;
    ros::init(argc,argv,"Example_Node");
    ros::NodeHandle nh;
    MyClass exampleClass = MyClass(&nh);
    ros::spin();
    return 1;
}