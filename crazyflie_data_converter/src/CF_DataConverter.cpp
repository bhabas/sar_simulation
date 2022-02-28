#include "CF_DataConverter.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"CF_DataConverter_Node");
    ros::NodeHandle nh;
    CF_DataConverter CF_DC = CF_DataConverter(&nh);
    ros::spin();
    return 0;
}