#include "SAR_DataConverter.h"

#define formatBool(b) ((b) ? "True" : "False")


// =========================
//     LOGGING FUNCTIONS
// =========================

bool SAR_DataConverter::DataLogging_Callback(sar_msgs::Logging_CMD::Request &req, sar_msgs::Logging_CMD::Response &res)
{
    switch(req.Logging_CMD){
        case 0: // CREATE CSV WHEN ACTIVATED
            Logging_Flag = false;
            fPtr = fopen(req.filePath.c_str(), "w");
            create_CSV();
            break;


        case 1: // TURN ON/OFF LOGGING
            Logging_Flag = true;
            fPtr = fopen(req.filePath.c_str(), "a");
            break;

        case 2: // CAP CSV W/ FLIP,IMPACT,MISC DATA
            Logging_Flag = false;

            fPtr = fopen(req.filePath.c_str(), "a");
            error_string = req.error_string;
            append_CSV_blank();
            append_CSV_misc();
            append_CSV_Rot();
            append_CSV_impact();
            append_CSV_blank();
            break;

    }



    return 1;
}


void SAR_DataConverter::LoggingLoop()
{

    ros::Rate rate(LOGGING_RATE);

    
    while(ros::ok)
    {   
        if(Logging_Flag == true)
        {
            append_CSV_states();
        }
        rate.sleep();
    }


}

void SAR_DataConverter::create_CSV()
{  
    // POLICY DATA
    fprintf(fPtr,"k_ep,k_run,");
    fprintf(fPtr,"t,");
    fprintf(fPtr,"Policy_Trg_Action,Policy_Rot_Action,");
    fprintf(fPtr,"mu,sigma,policy,");


    // STATE DATA
    fprintf(fPtr,"x,y,z,");
    fprintf(fPtr,"vx,vy,vz,");
    fprintf(fPtr,"D_perp,Tau,Tau_est,");
    fprintf(fPtr,"Theta_x,Theta_x_est,Theta_y,Theta_y_est,");
    fprintf(fPtr,"Rot_flag,impact_flag,");


    //  MISC STATE DATA
    fprintf(fPtr,"eul_x,eul_y,eul_z,");
    fprintf(fPtr,"wx,wy,wz,");
    fprintf(fPtr,"qx,qy,qz,qw,");
    fprintf(fPtr,"F_thrust,Mx,My,Mz,");

    // SETPOINT VALUES
    fprintf(fPtr,"x_d.x,x_d.y,x_d.z,");
    fprintf(fPtr,"v_d.x,v_d.y,v_d.z,");
    fprintf(fPtr,"a_d.x,a_d.y,a_d.z,");

    // MISC VALUES
    fprintf(fPtr,"Error");
    fprintf(fPtr,"\n");


    fprintf(fPtr,"# DATA_TYPE: %s, ",DATA_TYPE.c_str());
    fprintf(fPtr,"SAR_SETTINGS: {Policy_Type: %s, SAR_Type: %s, SAR_Config: %s}, ",POLICY_TYPE.c_str(),SAR_Type.c_str(),SAR_Config.c_str());
    fprintf(fPtr,"PLANE_SETTINGS: {Plane_Config: %s}, ",Plane_Config.c_str());
    fprintf(fPtr,"\n");

    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_states()
{
    // POLICY DATA
    fprintf(fPtr,"%u,%u,",k_ep,k_run);                          // k_ep,k_run
    fprintf(fPtr,"%.3f,",(Time-Time_start).toSec());            // t
    fprintf(fPtr,"%.3f,%.3f,",Policy_Trg_Action,Policy_Rot_Action);       // Policy_Trg_Action,Policy_Rot_Action
    fprintf(fPtr,"--,--,--,");                                  // mu,sigma,policy


    // STATE DATA
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Pose.position.x,Pose.position.y,Pose.position.z);    // x,y,z
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Twist.linear.x,Twist.linear.y,Twist.linear.z);       // vx,vy,vz
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",D_perp,Tau,Tau_est);                    // Tau,Theta_x,Theta_y,D_perp
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,%.3f,",Theta_x,Theta_x_est,Theta_y,Theta_y_est);                    // Tau_est,Theta_x_est,Theta_y_est
    fprintf(fPtr,"%s,%s,",formatBool(Rot_flag),formatBool(impact_flag));               // Rot_flag,impact_flag


    // MISC STATE DATA
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Eul.x,Eul.y,Eul.z);                                  // eul_x,eul_y,eul_z
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Twist.angular.x,Twist.angular.y,Twist.angular.z);    // wx,wy,wz
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w); // qx,qy,qz,qw
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",FM[0],FM[1],FM[2],FM[3]);                       // F_thrust,Mx,My,Mz


    // SETPOINT VALUES
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",x_d.x,x_d.y,x_d.z);  // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",v_d.x,v_d.y,v_d.z);  // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",a_d.x,a_d.y,a_d.z);  // a_d.x,a_d.y,a_d.z


    // MISC VALUES
    fprintf(fPtr,"--"); // Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_misc()
{
    // POLICY DATA
    fprintf(fPtr,"%u,%u,",k_ep,k_run);  // k_ep,k_run
    fprintf(fPtr,"% 6.2f,",Plane_Angle);                // --
    fprintf(fPtr,"%u,--,",n_rollouts);  // n_rollouts,--
    fprintf(fPtr,"[%.3f %.3f],[%.3f %.3f],[%.3f %.3f],",mu[0],mu[1],sigma[0],sigma[1],policy[0],policy[1]); // mu,sigma,policy


    // STATE DATA
    fprintf(fPtr,"--,--,--,");                                  // x,y,z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",vel_d[0],vel_d[1],vel_d[2]); // vel_d.x,vel_d.y,vel_d.z
    fprintf(fPtr,"--,--,--,");                                  // D_perp, Tau, Tau_est
    fprintf(fPtr,"--,--,--,--,");                               // Theta_x,Theta_x_est,Theta_y,Theta_y_est,
    fprintf(fPtr,"%.2f,[%.2f %.2f %.2f %.2f %.2f],",reward,reward_vals[0],reward_vals[1],reward_vals[2],reward_vals[3],reward_vals[4]); // Rot_flag,impact_flag


    // MISC STATE DATA
    fprintf(fPtr,"--,--,--,");      // eul_x,eul_y,eul_z
    fprintf(fPtr,"--,--,--,");      // wx,wy,wz
    fprintf(fPtr,"--,--,--,--,");   // qx,qy,qz,qw
    fprintf(fPtr,"--,--,--,--,");   // F_thrust,Mx,My,Mz


    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");  // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"--,--,--,");  // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"--,--,--,");  // a_d.x,a_d.y,a_d.z


    // MISC VALUES
    fprintf(fPtr,"%s",error_string.c_str()); // Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_Rot()
{
    fprintf(fPtr,"%u,%u,",k_ep,k_run);                          // k_ep,k_run
    fprintf(fPtr,"%.3f,",(Time_trg-Time_start).toSec());         // t
    fprintf(fPtr,"%.3f,%.3f,",Policy_Trg_Action_trg,Policy_Rot_Action_trg); // Policy_Trg_Action,Policy_Rot_Action
    fprintf(fPtr,"--,--,--,");                                  // mu,sigma,policy

    // // INTERNAL STATE ESTIMATES (CF)
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose_trg.position.x,Pose_trg.position.y,Pose_trg.position.z);   // x,y,z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_trg.linear.x,Twist_trg.linear.y,Twist_trg.linear.z);      // vx,vy,vz
    fprintf(fPtr,"%.3f,%.3f,--,",D_perp_trg,Tau_trg);                                             // D_perp,Tau,Tau_est
    fprintf(fPtr,"%.3f,--,%.3f,--,",Theta_x_trg,Theta_y_trg);                                     // Tau_est,Theta_x_est,Theta_y_est
    fprintf(fPtr,"%s,--,",formatBool(Rot_flag));                                               // Rot_flag,impact_flag



    fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul_trg.x,Eul_trg.y,Eul_trg.z);                                 // eul_x,eul_y,eul_z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_trg.angular.x,Twist_trg.angular.y,Twist_trg.angular.z);   // wx,wy,wz
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose_trg.orientation.x,Pose_trg.orientation.y,Pose_trg.orientation.z,Pose_trg.orientation.w); // qx,qy,qz,qw
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",FM_trg[0],FM_trg[1],FM_trg[2],FM_trg[3]);                   // F_thrust,Mx,My,Mz

    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,"); // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"--,--,--,"); // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"--,--,--,"); // a_d.x,a_d.y,a_d.z


    // MISC VALUES
    fprintf(fPtr,"%s","Rot Data"); // Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_impact()
{
    // POLICY DATA
    fprintf(fPtr,"%u,%u,",k_ep,k_run);                      // k_ep,k_run
    fprintf(fPtr,"%.3f,",(Time_impact-Time_start).toSec()); // t
    fprintf(fPtr,"--,--,");                                 // Policy_Trg_Action,Policy_Rot_Action
    fprintf(fPtr,"--,--,--,");                              // mu,sigma,policy


    // STATE DATA
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose_impact.position.x,Pose_impact.position.y,Pose_impact.position.z);   // x,y,z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_impact.linear.x,Twist_impact.linear.y,Twist_impact.linear.z);      // vx,vy,vz
    fprintf(fPtr,"--,--,--,");                                                                              // D_perp,Tau,Tau_est,
    fprintf(fPtr,"%u,%u,%u,%u,",Pad1_Contact,Pad2_Contact,Pad3_Contact,Pad4_Contact);   // Theta_x,Theta_x_est,Theta_y,Theta_y_est,
    fprintf(fPtr,"%s,%s,",formatBool(BodyContact_flag),formatBool(impact_flag));        // Rot_flag,impact_flag


    // MISC STATE DATA
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul_impact.x,Eul_impact.y,Eul_impact.z);                                 // eul_x,eul_y,eul_z
    fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_impact.angular.x,Twist_impact.angular.y,Twist_impact.angular.z);   // wx,wy,wz
    fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose_impact.orientation.x,Pose_impact.orientation.y,Pose_impact.orientation.z,Pose_impact.orientation.w); // qx,qy,qz,qw
    fprintf(fPtr,"%u,--,--,--,",Pad_Connect_Sum); // F_thrust,Mx,My,Mz


    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");
    fprintf(fPtr,"--,--,--,");
    fprintf(fPtr,"--,--,--,");


    // MISC VALUES
    fprintf(fPtr,"%s","Impact Data");
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_blank()
{
    fprintf(fPtr,"\n");
    fflush(fPtr);
}

