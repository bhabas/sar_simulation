#include "SAR_DataConverter.h"

// CONVERT QUATERNION TO EULER ANGLES (YZX NOTATION)
void SAR_DataConverter::quat2euler(float quat[], float eul[]){

    float R11,R21,R31,R22,R23;
    float phi,theta,psi; // (phi=>x,theta=>y,psi=>z)

    // CALC NEEDED ROTATION MATRIX COMPONENTS FROM QUATERNION
    R11 = 1.0 - 2.0*(pow(quat[1],2) + pow(quat[2],2) );
    R21 = 2.0*(quat[0]*quat[1] + quat[2]*quat[3]);
    R31 = 2.0*(quat[0]*quat[2] - quat[1]*quat[3]);

    R22 = 1.0 - 2.0*( pow(quat[0],2) + pow(quat[2],2) );
    R23 = 2.0*(quat[1]*quat[2] - quat[0]*quat[3]);

    // CONVERT ROTATION MATRIX COMPONENTS TO EULER ANGLES (YZX NOTATION)
    phi = atan2(-R23, R22);
    theta = atan2(-R31, R11);
    psi = asin(R21);

    eul[0] = phi;   // X-axis
    eul[1] = theta; // Y-axis
    eul[2] = psi;   // Z-axis
}


void SAR_DataConverter::LoadParams()
{
    // QUAD SETTINGS
    ros::param::get("/QUAD_SETTINGS/CF_Type",SAR_Type);
    ros::param::get("/QUAD_SETTINGS/CF_Config",SAR_Config);
    ros::param::get("/QUAD_SETTINGS/Policy_Type",POLICY_TYPE);

    GZ_Model_Name = "crazyflie_" + SAR_Config;
    std::string CF_Type_str = "/CF_Type/" + SAR_Type;
    std::string CF_Config_str = "/Config/" + SAR_Config;

    // // PLANE SETTINGS
    ros::param::get("/PLANE_SETTINGS/Plane_Model",Plane_Model);
    ros::param::get("/PLANE_SETTINGS/Plane_Angle",Plane_Angle_0);
    ros::param::get("/PLANE_SETTINGS/Pos_X",Plane_Pos_0.x);
    ros::param::get("/PLANE_SETTINGS/Pos_Y",Plane_Pos_0.y);
    ros::param::get("/PLANE_SETTINGS/Pos_Z",Plane_Pos_0.z);


    // COLLECT MODEL PARAMETERS
    ros::param::get(CF_Type_str + CF_Config_str + "/Mass",Mass);
    ros::param::get(CF_Type_str + CF_Config_str + "/Ixx",Ixx);
    ros::param::get(CF_Type_str + CF_Config_str + "/Iyy",Iyy);
    ros::param::get(CF_Type_str + CF_Config_str + "/Izz",Izz);

    // DEBUG SETTINGS
    ros::param::get("/DATA_TYPE",DATA_TYPE);
    ros::param::get("/SIM_SETTINGS/Sim_Speed",SIM_SPEED);
    ros::param::get("/SIM_SETTINGS/Sim_Slowdown_Speed",SIM_SLOWDOWN_SPEED);
    ros::param::get("/SIM_SETTINGS/Landing_Slowdown_Flag",LANDING_SLOWDOWN_FLAG);

    ros::param::get("/CF_DC_SETTINGS/Logging_Rate",LOGGING_RATE);

    // COLLECT CTRL GAINS
    ros::param::get(CF_Type_str + "/CtrlGains/P_kp_xy",P_kp_xy);
    ros::param::get(CF_Type_str + "/CtrlGains/P_kd_xy",P_kd_xy);
    ros::param::get(CF_Type_str + "/CtrlGains/P_ki_xy",P_ki_xy);

    ros::param::get(CF_Type_str + "/CtrlGains/P_kp_z",P_kp_z);
    ros::param::get(CF_Type_str + "/CtrlGains/P_kd_z",P_kd_z);
    ros::param::get(CF_Type_str + "/CtrlGains/P_ki_z",P_ki_z);

    ros::param::get(CF_Type_str + "/CtrlGains/R_kp_xy",R_kp_xy);
    ros::param::get(CF_Type_str + "/CtrlGains/R_kd_xy",R_kd_xy);
    ros::param::get(CF_Type_str + "/CtrlGains/R_ki_xy",R_ki_xy);
    
    ros::param::get(CF_Type_str + "/CtrlGains/R_kp_z",R_kp_z);
    ros::param::get(CF_Type_str + "/CtrlGains/R_kd_z",R_kd_z);
    ros::param::get(CF_Type_str + "/CtrlGains/R_ki_z",R_ki_z);

    if(DATA_TYPE.compare("SIM") == 0)
    {
        ros::param::set("/use_sim_time",true);
    }
    else
    {
        ros::param::set("/use_sim_time",false);
    }

}

void SAR_DataConverter::MainInit()
{
    adjustSimSpeed(0.33);
    LoadParams();

}

void SAR_DataConverter::MainLoop()
{

    MainInit();
    int loopRate = 50;     // [Hz]
    ros::Rate rate(loopRate);

    
    while(ros::ok)
    {   
        // printf("Tick %d\n",tick);
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