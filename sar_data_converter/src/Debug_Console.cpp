#include "SAR_DataConverter.h"


void SAR_DataConverter::ConsoleLoop() // MAIN CONTROLLER LOOP
{
    setlocale(LC_CTYPE,"C-UTF-8");
    set_escdelay(25);
    use_extended_names(TRUE);
    initscr();
    timeout(0); // Set getch() to non-blocking mode

    const int refresh_rate = 20; // 20 Hz
    const int delay_time_us = 1000000 / refresh_rate;

    while(SHOW_CONSOLE == true) {
        // Clear the screen buffer
        erase();

        mvprintw(0, 0,"t: %.4f",(Time-Time_start).toSec());
        mvprintw(1, 0,"SAR Type:   %s",SAR_Type.c_str());
        mvprintw(2, 0,"SAR Config: %s",SAR_Config.c_str());

        mvprintw(0, 30,"DataType: %s",DATA_TYPE.c_str());
        mvprintw(1, 30,"Plane Model: %s",Plane_Config.c_str());
        mvprintw(2, 30,"Plane Angle: % 6.2f",Plane_Angle_deg);


        mvprintw(4, 0,"==== Flags ====");
        mvprintw(5, 0,"Motorstop:     %u  Policy_Armed: %u  Pos_Ctrl:      %u  AngAccel_flag: %u",Motorstop_Flag,Policy_Armed_Flag,Pos_Ctrl_Flag,AngAccel_flag);
        mvprintw(6, 0,"SafeMode:      %u  Trg_flag:     %u  Vel_Ctrl:      %u  AttCtrl_Flag:  %u",SafeModeEnable,Trg_flag,Vel_Ctrl_Flag,AttCtrl_Flag);
        mvprintw(7, 0,"Tumbled:       %u  Impact_Flag:  %u  Sticky_Flag:   %u  Custom_Thrust: %u",Tumbled_Flag,Impact_flag,Sticky_Flag,CustomThrust_Flag);
        mvprintw(8, 0,"Tumble_Detect: %u  Cam_Active:   %u  Slowdown_Type: %u  Custom_PWM:    %u",Tumble_Detection,isCamActive,SLOWDOWN_TYPE,CustomPWM_Flag);
        
        mvprintw(10, 0,"============ World States ============");
        mvprintw(11, 0,"Pos [m]:          % 6.2f % 6.2f % 6.2f",Pose_B_O.position.x,Pose_B_O.position.y,Pose_B_O.position.z);
        mvprintw(12, 0,"Vel [m/s]:        % 6.2f % 6.2f % 6.2f",Twist_B_O.linear.x,Twist_B_O.linear.y,Twist_B_O.linear.z);
        mvprintw(13, 0,"Accel [m/s^2]:    % 6.2f % 6.2f % 6.2f",Accel_B_O.linear.x,Accel_B_O.linear.y,Accel_B_O.linear.z);
        mvprintw(14, 0,"Omega [rad/s]:    % 6.2f % 6.2f % 6.2f",Twist_B_O.angular.x,Twist_B_O.angular.y,Twist_B_O.angular.z);
        mvprintw(15, 0,"Ang Acc [rad/s^2]:% 6.2f % 6.2f % 6.2f",Accel_B_O.angular.x,Accel_B_O.angular.y,Accel_B_O.angular.z);

        mvprintw(10, 42,"========== Relative States ==========");
        // mvprintw(11, 42,"r_P_B [m]: % 6.2f % 6.2f % 6.2f",r_P_B.x,r_P_B.y,r_P_B.z);

        // mvprintw(12, 42,"D_perp:% 6.2f  V_perp:% 6.2f  V_tx:% 6.2f",D_perp,V_rel.z,V_rel.x);
        // mvprintw(14, 0,"Eul [deg]:    % 8.3f % 8.3f % 8.3f",Eul.x,Eul.y,Eul.z);
        // mvprintw(11, 42,"[V_mag, V_angle]:% 7.2f % 6.1f",Vel_mag,Phi);
        // mvprintw(12, 42,"[V_rel, V_angle_rel]:");
        // mvprintw(13, 42,"Acc_Mag [m/s^2]: % 7.2f",Acc_mag);






        mvprintw(17, 0,"==== Relative States ====");
        // mvprintw(18, 0,"D_perp:  % 7.3f  V_perp:      % 7.3f  V_tx:        % 7.3f",D_perp,V_rel.z,V_rel.x);
        mvprintw(19, 0,"Tau:     % 7.3f  Theta_x:     % 7.3f  Theta_y:     % 7.3f",Tau,Theta_x,Theta_y);
        mvprintw(20, 0,"Tau_est: % 7.3f  Theta_x_est: % 7.3f  Theta_y_est: % 7.3f",Tau_est,Theta_x_est,Theta_y_est);


        mvprintw(23, 0,"==== Policy: %s ====",POLICY_TYPE.c_str());
        if (strcmp(POLICY_TYPE.c_str(),"PARAM_OPTIM") == 0)
        {
            mvprintw(23, 0,"Tau_thr: % 7.3f \tMy: % 7.3f",Policy_Trg_Action,Policy_Rot_Action);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL_SB3") == 0)
        {
            mvprintw(23, 0,"Pol_Trg_Act_trg: % 7.3f \tPol_Rot_Act: % 7.3f ",Policy_Trg_Action,Policy_Rot_Action);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL_ONBOARD") == 0)
        {
            mvprintw(23, 0,"Pol_Trg_Act_trg: % 7.3f \tPol_Rot_Act: % 7.3f ",Policy_Trg_Action,Policy_Rot_Action);
        }

        mvprintw(25,0,"==== Rot Trigger Values ====");
        mvprintw(26,0,"Tau_trg:     % 7.3f \tPol_Trg_Act_trg:  % 7.3f ",Tau_trg,Policy_Trg_Action_trg);
        mvprintw(27,0,"\u03B8x_trg:     % 7.3f \tPol_Rot_Act_trg: % 7.3f ",Theta_x_trg,Policy_Rot_Action_trg);
        mvprintw(28,0,"D_perp_trg:  % 7.3f ",D_perp_trg);

        mvprintw(31,0,"==== Setpoints ====");
        mvprintw(32,0,"x_d: % 7.3f  % 7.3f  % 7.3f",x_d.x,x_d.y,x_d.z);
        mvprintw(33,0,"v_d: % 7.3f  % 7.3f  % 7.3f",v_d.x,v_d.y,v_d.z);
        mvprintw(34,0,"a_d: % 7.3f  % 7.3f  % 7.3f",a_d.x,a_d.y,a_d.z);

        mvprintw(36,0,"==== Controller Actions ====");
        mvprintw(37,0,"FM [N/N*mm]:       % 7.3f  % 7.3f  % 7.3f  % 7.3f",FM[0],FM[1],FM[2],FM[3]);
        mvprintw(38,0,"Motor Thrusts [g]: % 7.3f  % 7.3f  % 7.3f  % 7.3f",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
        mvprintw(39,0,"MS_PWM: %u  %u  %u  %u",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);


        mvprintw(41,0,"=== Controller Gains ====");
        mvprintw(42,0,"Kp_P: % 7.3f  % 7.3f  % 7.3f ",P_kp_xy,P_kp_xy,P_kp_z);
        mvprintw(42,37,"Kp_R: % 7.3f  % 7.3f  % 7.3f ",R_kp_xy,R_kp_xy,R_kp_z);

        mvprintw(43,0,"Kd_P: % 7.3f  % 7.3f  % 7.3f ",P_kd_xy,P_kd_xy,P_kd_z);
        mvprintw(43,37,"Kd_R: % 7.3f  % 7.3f  % 7.3f ",R_kd_xy,R_kd_xy,R_kd_z);

        mvprintw(44,0,"Ki_P: % 7.3f  % 7.3f  % 7.3f ",P_ki_xy,P_ki_xy,P_ki_z);
        mvprintw(44,37,"Ki_R: % 7.3f  % 7.3f  % 7.3f ",R_ki_xy,R_ki_xy,R_ki_z);
        mvprintw(45,0,"======\n");


        // Refresh the screen with the updated buffer
        refresh();

        // Sleep for the desired delay time
        usleep(delay_time_us);
    }

    // Clean up and close the ncurses library
    endwin();
}