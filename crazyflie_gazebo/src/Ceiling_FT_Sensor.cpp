#include "Ceiling_FT_Sensor.h"




int main(int argc, char **argv)
{
    ros::init(argc,argv,"Ceiling_FT_SensorNode");
    ros::NodeHandle nh;
    Ceiling_FT_Sensor Ceiling_FT_Init = Ceiling_FT_Sensor(&nh);
    ros::spin();
    return 1;
}