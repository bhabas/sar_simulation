#include "SAR_DataConverter.h"

void SAR_DataConverter::MainInit()
{
    adjustSimSpeed(0.33);

}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    int loopRate = 50;     // [Hz]
    ros::Rate rate(loopRate);

    
    while(ros::ok)
    {   
        printf("Tick %d\n",tick);
        tick++;
        rate.sleep();
    }


}

int main(int argc, char** argv)
{
    setlocale(LC_CTYPE,"C-UTF-8");
    ros::init(argc,argv,"CF_DataConverter_Node");
    ros::NodeHandle nh;
    SAR_DataConverter SAR_DC = SAR_DataConverter(&nh);
    ros::spin();
    return 0;
}