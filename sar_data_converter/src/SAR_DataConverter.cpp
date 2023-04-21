#include "SAR_DataConverter.h"


int main(int argc, char** argv)
{
    setlocale(LC_CTYPE,"C-UTF-8");
    ros::init(argc,argv,"CF_DataConverter_Node");
    ros::NodeHandle nh;
    SAR_DC DC = SAR_DC(&nh);
    ros::spin();
    return 0;
}