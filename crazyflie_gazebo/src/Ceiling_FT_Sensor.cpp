#include "Ceiling_FT_Sensor.h"


int main(int argc, char **argv)
{
    ros::init(argc,argv,"Example_Node");
    ros::NodeHandle nh;
    MyClass exampleClass = MyClass(&nh);
    ros::spin();
    return 1;
}