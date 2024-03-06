#include "SAR_DataConverter.h"




void SAR_DataConverter::MainInit()
{
    loadInitParams();
    adjustSimSpeed(SIM_SPEED);
    Time_start = ros::Time::now(); 

}

void SAR_DataConverter::CrazyswarmPingLoop()
{
    int loopRate = 5;     // [Hz]
    ros::Rate rate(loopRate);

    while(ros::ok())
    {
        // CONTINUOUSLY PING CRAZYSWARM FOR EXPERIMENT SETUP
        if(DATA_TYPE.compare("EXP") == 0)
        {
            sar_msgs::CTRL_Cmd cmd_msg;
            cmd_msg.cmd_type = 99;
            cmd_msg.cmd_vals.x = 0;
            cmd_msg.cmd_vals.y = 0;
            cmd_msg.cmd_vals.z = 0;
            cmd_msg.cmd_flag = 0;
            cmd_msg.cmd_rx = 1;

            for (int i = 0; i < 3; i++)
            {
                CMD_Output_Topic.publish(cmd_msg);
            }
        }
        rate.sleep();
    }
}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    const int refresh_rate = 50; // 20 Hz
    const int delay_time_us = 1000000 / refresh_rate;

    
    while(ros::ok)
    {   
        checkSlowdown();
        
        // PUBLISH ORGANIZED DATA
        Publish_StateData();
        Publish_TriggerData();
        Publish_ImpactData();
        Publish_MiscData();

        usleep(delay_time_us);
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