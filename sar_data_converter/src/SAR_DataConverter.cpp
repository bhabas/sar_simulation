#include "SAR_DataConverter.h"




void SAR_DataConverter::MainInit()
{
    loadInitParams();
    adjustSimSpeed(SIM_SPEED);
    Time_start = ros::Time::now(); 

}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    int loopRate = 1000;     // [Hz]
    ros::Rate rate(loopRate);

    
    while(ros::ok)
    {   
        checkSlowdown();
        
        // PUBLISH ORGANIZED DATA
        Publish_StateData();
        Publish_TriggerData();
        Publish_ImpactData();
        Publish_MiscData();

        rate.sleep();
    }


}

int main(int argc, char** argv)
{
    
    ros::init(argc,argv,"SAR_DataConverter_Node");
    ros::NodeHandle nh;
    SAR_DataConverter SAR_DC = SAR_DataConverter(&nh);
    ros::spin();
    return 0;
}