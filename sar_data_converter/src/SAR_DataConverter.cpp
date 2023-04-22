#include "SAR_DataConverter.h"




void SAR_DataConverter::MainInit()
{
    LoadParams();
    adjustSimSpeed(SIM_SPEED);
    Update_Landing_Surface_Pose(Plane_Pos_0.x,Plane_Pos_0.y,Plane_Pos_0.z,180.0);

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