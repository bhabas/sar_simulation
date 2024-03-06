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

        case 2: // CAP CSV W/ TRIGGER,IMPACT,MISC DATA
            Logging_Flag = false;

            fPtr = fopen(req.filePath.c_str(), "a");
            error_string = req.error_string;
            append_CSV_blank();
            append_CSV_misc();
            append_CSV_Trg();
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
    fprintf(fPtr,"K_ep,K_run,");
    fprintf(fPtr,"t,");
    fprintf(fPtr,"Trg_Action,Rot_Action,");
    fprintf(fPtr,"Mu,Sigma,Policy,");

    // STATE DATA
    fprintf(fPtr,"D_perp,Tau,Tau_CR,Theta_x,");
    fprintf(fPtr,"V_BO_Mag,V_BO_Angle,a_BO_Mag,");
    fprintf(fPtr,"V_BP_Mag,V_BP_Angle,Phi_PB,");
    fprintf(fPtr,"Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB,");

    // STATE DATA
    fprintf(fPtr,"r_BO.x,r_BO.y,r_BO.z,");
    fprintf(fPtr,"V_BO.x,V_BO.y,V_BO.z,");
    fprintf(fPtr,"a_BO.x,a_BO.y,a_BO.z,");


    //  MISC STATE DATA
    fprintf(fPtr,"Eul_BO.x,Eul_BO.y,Eul_BO.z,");
    fprintf(fPtr,"W_BO.x,W_BO.y,W_BO.z,");
    fprintf(fPtr,"AngAcc_BO.y,");
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
    // POLICY DATA
    fprintf(fPtr,"%u,%u,",K_ep,K_run);                                                                          // K_ep,K_run
    fprintf(fPtr,"% 5.3f,",(Time-Time_start).toSec());                                                          // t
    fprintf(fPtr,"% 5.3f,%.1f,",a_Trg,a_Rot);                                           // Trg_Action,Rot_Action
    fprintf(fPtr,"--,--,--,");                                                                                  // Mu,Sigma,Policy

    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",D_perp,Tau,Tau_CR,Theta_x);                                     // D_perp,Tau,Tau_CR,Theta_x
    fprintf(fPtr,"% 5.2f,% 7.2f,% 5.2f,",Vel_mag_B_O,Vel_angle_B_O,Accel_B_O_Mag);                              // V_BO_Mag,V_BO_Angle,a_BO_Mag
    fprintf(fPtr,"% 5.2f,% 7.2f,% 6.2f,",Vel_mag_B_P,Vel_angle_B_P,Eul_P_B.y);                                  // V_BP_Mag,V_BP_Angle,Phi_PB
    fprintf(fPtr,"%s,%s,%s,",formatBool(Trg_Flag),formatBool(Impact_Flag_Ext),formatBool(Impact_Flag_OB));      // Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB

    // STATE DATA
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Pose_B_O.position.x,Pose_B_O.position.y,Pose_B_O.position.z);          // r_BO.x,r_BO.y,r_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Twist_B_O.linear.x,Twist_B_O.linear.y,Twist_B_O.linear.z);             // V_BO.x,V_BO.y,V_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Accel_B_O.linear.x,Accel_B_O.linear.y,Accel_B_O.linear.z);             // a_BO.x,a_BO.y,a_BO.z
    
    // MISC STATE DATA
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Eul_B_O.x,Eul_B_O.y,Eul_B_O.z);                                        // Eul_BO.x,Eul_BO.y,Eul_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Twist_B_O.angular.x,Twist_B_O.angular.y,Twist_B_O.angular.z);          // W_BO.x,W_BO.y,W_BO.z
    fprintf(fPtr,"% 5.2f,",Accel_B_O.angular.y);                                                                // AngAcc_BO.y
    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",FM[0],FM[1],FM[2],FM[3]);                                       // F_thrust,Mx,My,Mz

    // SETPOINT VALUES
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",x_d.x,x_d.y,x_d.z);                                                    // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",v_d.x,v_d.y,v_d.z);                                                    // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",a_d.x,a_d.y,a_d.z);                                                    // a_d.x,a_d.y,a_d.z

    // MISC VALUES
    fprintf(fPtr,"--"); // Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_misc()
{
    // POLICY DATA
    fprintf(fPtr,"%u,%u,",K_ep,K_run);  // K_ep,K_run
    fprintf(fPtr,"--,"); 
    fprintf(fPtr,"%u,--,",n_rollouts);  // n_rollouts,--
    fprintf(fPtr,"[%.3f %.3f],[%.3f %.3f],[%.3f %.3f],",mu[0],mu[1],sigma[0],sigma[1],policy[0],policy[1]); // mu,sigma,policy

    // STATE DATA
    fprintf(fPtr,"% 6.2f,% 6.2f,% 6.2f,% 6.2f,",Plane_Angle_deg,Plane_Pos.x,Plane_Pos.y,Plane_Pos.z); // D_perp,Tau,Tau_CR,Theta_x
    fprintf(fPtr,"--,--,--," ); // V_BO_Mag,V_BO_Angle,a_BO_Mag
    fprintf(fPtr,"--,--,--," ); // V_BP_Mag,V_BP_Angle,Phi_PB
    fprintf(fPtr,"--,--,--," ); // Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB

    // STATE DATA
    fprintf(fPtr,"--,--,--,"); // r_BO.x,r_BO.y,r_BO.z
    fprintf(fPtr,"--,--,--,"); // V_BO.x,V_BO.y,V_BO.z
    fprintf(fPtr,"--,--,--,"); // a_BO.x,a_BO.y,a_BO.z

    // MISC STATE DATA
    fprintf(fPtr,"--,--,--,");  // Eul_BO.x,Eul_BO.y,Eul_BO.z
    fprintf(fPtr,"--,--,--,");  // W_BO.x,W_BO.y,W_BO.z
    fprintf(fPtr,"--,");        // AngAcc_BO.y
    fprintf(fPtr,"--,--,--,--,"); // F_thrust,Mx,My,Mz

    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,"); // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"--,--,--,"); // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"--,--,--,"); // a_d.x,a_d.y,a_d.z


    // MISC VALUES
    fprintf(fPtr,"%s,",error_string.c_str()); // Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_Trg()
{

    // POLICY DATA
    fprintf(fPtr,"%u,%u,",K_ep,K_run);                                                                          // K_ep,K_run
    fprintf(fPtr,"% 5.3f,",(Time_trg-Time_start).toSec());                                                          // t
    fprintf(fPtr,"% 5.3f,%.1f,",a_Trg_trg,a_Rot_trg);                                           // Trg_Action,Rot_Action
    fprintf(fPtr,"--,--,--,");                                                                                  // Mu,Sigma,Policy

    fprintf(fPtr,"% 6.3f,% 6.3f,% 6.3f,% 6.3f,",D_perp_trg,Tau_trg,Tau_CR_trg,Theta_x_trg);                                     // D_perp,Tau,Tau_CR,Theta_x
    fprintf(fPtr,"% 5.2f,% 7.2f,--,",Vel_mag_B_O_trg,Vel_angle_B_O_trg);                              // V_BO_Mag,V_BO_Angle,a_BO_Mag
    fprintf(fPtr,"% 5.2f,% 7.2f,--,",Vel_mag_B_P_trg,Vel_angle_B_P_trg);                                  // V_BP_Mag,V_BP_Angle,Phi_PB
    fprintf(fPtr,"%s,--,--,",formatBool(Trg_Flag));      // Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB

    // STATE DATA
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Pose_B_O_trg.position.x,Pose_B_O_trg.position.y,Pose_B_O_trg.position.z);          // r_BO.x,r_BO.y,r_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Twist_B_O_trg.linear.x,Twist_B_O_trg.linear.y,Twist_B_O_trg.linear.z);             // V_BO.x,V_BO.y,V_BO.z
    fprintf(fPtr,"--,--,--,");             // a_BO.x,a_BO.y,a_BO.z
    
    // MISC STATE DATA
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Eul_B_O_trg.x,Eul_B_O_trg.y,Eul_B_O_trg.z);                                        // Eul_BO.x,Eul_BO.y,Eul_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Twist_B_O_trg.angular.x,Twist_B_O_trg.angular.y,Twist_B_O_trg.angular.z);          // W_BO.x,W_BO.y,W_BO.z
    fprintf(fPtr,"--,");                                                                // AngAcc_BO.x,AngAcc_BO.y,AngAcc_BO.z
    fprintf(fPtr,"--,--,--,--,");                                       // F_thrust,Mx,My,Mz

    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");                                                    // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"--,--,--,");                                                    // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"--,--,--,");                                                    // a_d.x,a_d.y,a_d.z

    // MISC VALUES
    fprintf(fPtr,"%s","Trigger Data"); // Error
    fprintf(fPtr,"\n");
    fflush(fPtr);

}

void SAR_DataConverter::append_CSV_impact()
{
    // POLICY DATA
    fprintf(fPtr,"%u,%u,",K_ep,K_run);                              // K_ep,K_run
    fprintf(fPtr,"% 5.3f,",(Time_impact_Ext-Time_start).toSec());   // t_Impact
    fprintf(fPtr,"--,%u,",Pad_Connections);                         // --,Pad_Connections
    fprintf(fPtr,"%s,%s,%s,",formatBool(BodyContact_Flag),formatBool(ForelegContact_Flag),formatBool(HindlegContact_Flag));                                                                                  // Mu,Sigma,Policy

    fprintf(fPtr,"%u,%u,%u,%u,",Pad1_Contact,Pad2_Contact,Pad3_Contact,Pad3_Contact);   // Pad1_Contact,Pad2_Contact,Pad3_Contact,Pad4_Contact
    fprintf(fPtr,"--,--,--,");                                                          // V_BO_Mag,V_BO_Angle,a_BO_Mag
    fprintf(fPtr,"--,% 7.2f,% 7.2f,",Rot_Sum_impact_Ext,Eul_P_B_impact_Ext.y);                                 // V_BP_Mag,V_BP_Angle,Phi_PB
    fprintf(fPtr,"--,%s,%s,",formatBool(Impact_Flag_Ext),formatBool(Impact_Flag_OB));   // Trg_Flag,Impact_Flag_Ext,Impact_Flag_OB

    // STATE DATA
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Pose_B_O_impact_Ext.position.x,Pose_B_O_impact_Ext.position.y,Pose_B_O_impact_Ext.position.z);     // r_BO.x,r_BO.y,r_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Twist_B_P_impact_Ext.linear.x,Twist_B_P_impact_Ext.linear.y,Twist_B_P_impact_Ext.linear.z);        // V_BO.x,V_BO.y,V_BO.z
    fprintf(fPtr,"--,--,--,");                                                                                                              // a_BO.x,a_BO.y,a_BO.z
    
    // MISC STATE DATA
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Eul_B_O_impact_Ext.x,Eul_B_O_impact_Ext.y,Eul_B_O_impact_Ext.z);                                   // Eul_BO.x,Eul_BO.y,Eul_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,",Twist_B_P_impact_Ext.angular.x,Twist_B_P_impact_Ext.angular.y,Twist_B_P_impact_Ext.angular.z);     // W_BO.x,W_BO.y,W_BO.z
    fprintf(fPtr,"--,");                                                                                                                    // AngAcc_BO.x,AngAcc_BO.y,AngAcc_BO.z
    fprintf(fPtr,"% 5.2f,% 5.2f,% 5.2f,% 5.2f,",Impact_Magnitude,Force_Impact_x,Force_Impact_y,Force_Impact_z);                             // F_thrust,Mx,My,Mz

    // SETPOINT VALUES
    fprintf(fPtr,"--,--,--,");                                                    // x_d.x,x_d.y,x_d.z
    fprintf(fPtr,"--,--,--,");                                                    // v_d.x,v_d.y,v_d.z
    fprintf(fPtr,"--,--,--,");                                                    // a_d.x,a_d.y,a_d.z

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

