#include "SAR_DataConverter.h"




void SAR_DataConverter::MainInit()
{
    LoadParams();
    adjustSimSpeed(SIM_SPEED);
    Time_start = ros::Time::now();
    BodyCollision_str = GZ_Model_Name + "::crazyflie_Base_Model::crazyflie_body::body_collision"; // TODO: CHANGE TO SAR FORMAT
    // Update_Landing_Surface_Pose(Plane_Pos.x,Plane_Pos.y,Plane_Pos.z,Plane_Angle);
    isInit = true;

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