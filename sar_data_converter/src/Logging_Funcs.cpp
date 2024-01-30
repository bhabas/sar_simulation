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
            std::cout << "CSV Created" << std::endl;
            break;


        case 1: // TURN ON/OFF LOGGING
            Logging_Flag = true;
            fPtr = fopen(req.filePath.c_str(), "a");
            std::cout << "Logging Started" << std::endl;
            break;

        case 2: // CAP CSV W/ TRIGGER,IMPACT,MISC DATA
            Logging_Flag = false;

            fPtr = fopen(req.filePath.c_str(), "a");
            error_string = req.error_string;
            append_CSV_blank();
            append_CSV_misc();
            append_CSV_Rot();
            append_CSV_impact();
            append_CSV_blank();

            std::cout << "CSV Capped" << std::endl;
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
    fprintf(fPtr,"K_ep,K_run,");
    fprintf(fPtr,"t,");
    fprintf(fPtr,"Trg_Action,Rot_Action,");
    fprintf(fPtr,"Mu,Sigma,Policy,");


    // STATE DATA
    fprintf(fPtr,"r_BO.x,r_BO.y,r_BO.z,");
    fprintf(fPtr,"V_BO.x,V_BO.y,V_BO.z,");
    fprintf(fPtr,"a_BO.x,a_BO.y,a_BO.z,");
    fprintf(fPtr,"V_BO_Mag,V_BO_Angle,a_BO_Mag,");
    fprintf(fPtr,"V_BP_Mag,V_BP_Angle,Phi_PB,");


    fprintf(fPtr,"D_perp,Tau,Tau_CR,Theta_x,");
    fprintf(fPtr,"Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB,");


    //  MISC STATE DATA
    fprintf(fPtr,"Eul_BO.x,Eul_BO.y,Eul_BO.z,");
    fprintf(fPtr,"W_BO.x,W_BO.y,W_BO.z,");
    fprintf(fPtr,"AngAcc_BO.x,AngAcc_BO.y,AngAcc_BO.z,");
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
    fprintf(fPtr,"LEG_SETTINGS: {L_eff: %.3f, Gamma_eff: %.0f, K_Pitch: %.1f, K_Yaw: %.1f}, ",L_eff,Gamma_eff,K_Pitch,K_Yaw);
    fprintf(fPtr,"\n");

    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_states()
{
    // // POLICY DATA
    // fprintf(fPtr,"%u,%u,",K_ep,K_run);                          // K_ep,K_run
    // fprintf(fPtr,"%.3f,",(Time-Time_start).toSec());            // t
    // fprintf(fPtr,"%.3f,%.3f,",Policy_Trg_Action,Policy_Rot_Action);       // Policy_Trg_Action,Policy_Rot_Action
    // fprintf(fPtr,"--,--,--,");                                  // mu,sigma,policy


    // // STATE DATA
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Pose.position.x,Pose.position.y,Pose.position.z);    // x,y,z
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Twist.linear.x,Twist.linear.y,Twist.linear.z);       // vx,vy,vz
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",D_perp,Tau,Tau_Cam);                    // Tau,Theta_x,Theta_y,D_perp
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,%.3f,",Theta_x,Theta_x_Cam,Theta_y,Theta_y_Cam);                    // Tau_Cam,Theta_x_Cam,Theta_y_Cam
    // fprintf(fPtr,"%s,%s,",formatBool(Trg_Flag),formatBool(Impact_Flag_Ext));               // Trg_Flag,Impact_Flag_Ext


    // // MISC STATE DATA
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Eul_B_O.x,Eul_B_O.y,Eul_B_O.z);                                  // eul_x,eul_y,eul_z
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",Twist.angular.x,Twist.angular.y,Twist.angular.z);    // wx,wy,wz
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",Pose.orientation.x,Pose.orientation.y,Pose.orientation.z,Pose.orientation.w); // qx,qy,qz,qw
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",FM[0],FM[1],FM[2],FM[3]);                       // F_thrust,Mx,My,Mz


    // // SETPOINT VALUES
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",x_d.x,x_d.y,x_d.z);  // x_d.x,x_d.y,x_d.z
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",v_d.x,v_d.y,v_d.z);  // v_d.x,v_d.y,v_d.z
    // fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,",a_d.x,a_d.y,a_d.z);  // a_d.x,a_d.y,a_d.z


    // // MISC VALUES
    // fprintf(fPtr,"--"); // Error
    // fprintf(fPtr,"\n");
    // fflush(fPtr);

}

void SAR_DataConverter::append_CSV_misc()
{
    // // POLICY DATA
    // fprintf(fPtr,"%u,%u,",K_ep,K_run);  // K_ep,K_run
    // fprintf(fPtr,"% 6.2f,",Plane_Angle_deg);                // --
    // fprintf(fPtr,"%u,--,",n_rollouts);  // n_rollouts,--
    // fprintf(fPtr,"[%.3f %.3f],[%.3f %.3f],[%.3f %.3f],",mu[0],mu[1],sigma[0],sigma[1],policy[0],policy[1]); // mu,sigma,policy


    // // STATE DATA
    // fprintf(fPtr,"--,--,--,");                                  // x,y,z
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",vel_d[0],vel_d[1],vel_d[2]); // vel_d.x,vel_d.y,vel_d.z
    // fprintf(fPtr,"--,--,--,");                                  // D_perp, Tau, Tau_Cam
    // fprintf(fPtr,"--,--,--,--,");                               // Theta_x,Theta_x_Cam,Theta_y,Theta_y_Cam,
    // fprintf(fPtr,"%.2f,[%.2f %.2f %.2f %.2f %.2f],",reward,reward_vals[0],reward_vals[1],reward_vals[2],reward_vals[3],reward_vals[4]); // Trg_Flag,Impact_Flag_Ext


    // // MISC STATE DATA
    // fprintf(fPtr,"--,--,--,");      // eul_x,eul_y,eul_z
    // fprintf(fPtr,"--,--,--,");      // wx,wy,wz
    // fprintf(fPtr,"--,--,--,--,");   // qx,qy,qz,qw
    // fprintf(fPtr,"--,--,--,--,");   // F_thrust,Mx,My,Mz


    // // SETPOINT VALUES
    // fprintf(fPtr,"--,--,--,");  // x_d.x,x_d.y,x_d.z
    // fprintf(fPtr,"--,--,--,");  // v_d.x,v_d.y,v_d.z
    // fprintf(fPtr,"--,--,--,");  // a_d.x,a_d.y,a_d.z


    // // MISC VALUES
    // fprintf(fPtr,"%s",error_string.c_str()); // Error
    // fprintf(fPtr,"\n");
    // fflush(fPtr);

}

void SAR_DataConverter::append_CSV_Rot()
{
    // fprintf(fPtr,"%u,%u,",K_ep,K_run);                          // K_ep,K_run
    // fprintf(fPtr,"%.3f,",(Time_trg-Time_start).toSec());         // t
    // fprintf(fPtr,"%.3f,%.3f,",Policy_Trg_Action_trg,Policy_Rot_Action_trg); // Policy_Trg_Action,Policy_Rot_Action
    // fprintf(fPtr,"--,--,--,");                                  // mu,sigma,policy

    // // // INTERNAL STATE ESTIMATES (CF)
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose_trg.position.x,Pose_trg.position.y,Pose_trg.position.z);   // x,y,z
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_trg.linear.x,Twist_trg.linear.y,Twist_trg.linear.z);      // vx,vy,vz
    // fprintf(fPtr,"%.3f,%.3f,--,",D_perp_trg,Tau_trg);                                             // D_perp,Tau,Tau_Cam
    // fprintf(fPtr,"%.3f,--,%.3f,--,",Theta_x_trg,Theta_y_trg);                                     // Tau_Cam,Theta_x_Cam,Theta_y_Cam
    // fprintf(fPtr,"%s,--,",formatBool(Trg_Flag));                                               // Trg_Flag,Impact_Flag_Ext



    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul_trg.x,Eul_trg.y,Eul_trg.z);                                 // eul_x,eul_y,eul_z
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_trg.angular.x,Twist_trg.angular.y,Twist_trg.angular.z);   // wx,wy,wz
    // fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose_trg.orientation.x,Pose_trg.orientation.y,Pose_trg.orientation.z,Pose_trg.orientation.w); // qx,qy,qz,qw
    // fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",FM_trg[0],FM_trg[1],FM_trg[2],FM_trg[3]);                   // F_thrust,Mx,My,Mz

    // // SETPOINT VALUES
    // fprintf(fPtr,"--,--,--,"); // x_d.x,x_d.y,x_d.z
    // fprintf(fPtr,"--,--,--,"); // v_d.x,v_d.y,v_d.z
    // fprintf(fPtr,"--,--,--,"); // a_d.x,a_d.y,a_d.z


    // // MISC VALUES
    // fprintf(fPtr,"%s","Rot Data"); // Error
    // fprintf(fPtr,"\n");
    // fflush(fPtr);

}

void SAR_DataConverter::append_CSV_impact()
{
    // // POLICY DATA
    // fprintf(fPtr,"%u,%u,",K_ep,K_run);                      // K_ep,K_run
    // fprintf(fPtr,"%.3f,",(Time_impact-Time_start).toSec()); // t
    // fprintf(fPtr,"--,--,");                                 // Policy_Trg_Action,Policy_Rot_Action
    // fprintf(fPtr,"--,--,--,");                              // mu,sigma,policy


    // // STATE DATA
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Pose_impact.position.x,Pose_impact.position.y,Pose_impact.position.z);   // x,y,z
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_impact.linear.x,Twist_impact.linear.y,Twist_impact.linear.z);      // vx,vy,vz
    // fprintf(fPtr,"--,--,--,");                                                                              // D_perp,Tau,Tau_Cam,
    // fprintf(fPtr,"%u,%u,%u,%u,",Pad1_Contact,Pad2_Contact,Pad3_Contact,Pad4_Contact);   // Theta_x,Theta_x_Cam,Theta_y,Theta_y_Cam,
    // fprintf(fPtr,"%s,%s,",formatBool(BodyContact_Flag),formatBool(Impact_Flag_Ext));        // Trg_Flag,Impact_Flag_Ext


    // // MISC STATE DATA
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Eul_impact.x,Eul_impact.y,Eul_impact.z);                                 // eul_x,eul_y,eul_z
    // fprintf(fPtr,"%.3f,%.3f,%.3f,",Twist_impact.angular.x,Twist_impact.angular.y,Twist_impact.angular.z);   // wx,wy,wz
    // fprintf(fPtr,"%.3f,%.3f,%.3f,%.3f,",Pose_impact.orientation.x,Pose_impact.orientation.y,Pose_impact.orientation.z,Pose_impact.orientation.w); // qx,qy,qz,qw
    // fprintf(fPtr,"%u,--,--,--,",Pad_Connections); // F_thrust,Mx,My,Mz


    // // SETPOINT VALUES
    // fprintf(fPtr,"--,--,--,");
    // fprintf(fPtr,"--,--,--,");
    // fprintf(fPtr,"--,--,--,");


    // // MISC VALUES
    // fprintf(fPtr,"%s","Impact Data");
    // fprintf(fPtr,"\n");
    // fflush(fPtr);

}

void SAR_DataConverter::append_CSV_blank()
{
    fprintf(fPtr,"\n");
    fflush(fPtr);
}

