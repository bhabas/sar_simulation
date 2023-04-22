#include "SAR_DataConverter.h"




void SAR_DataConverter::MainInit()
{
    LoadParams();
    adjustSimSpeed(SIM_SPEED);

}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    int loopRate = 500;     // [Hz]
    ros::Rate rate(loopRate);

    
    while(ros::ok)
    {   
        // PUBLISH ORGANIZED DATA
        Publish_StateData();
        Publish_FlipData();
        Publish_ImpactData();
        Publish_MiscData();


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