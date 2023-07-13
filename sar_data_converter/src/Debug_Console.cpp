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

        mvprintw(0, 0,"t: %.4f V: %.3f\t  DataType: %s",(Time-Time_start).toSec(),V_battery,DATA_TYPE.c_str());
        mvprintw(1, 0,"SAR Type:   %s\t  Plane Model:  %s",SAR_Type.c_str(),Plane_Model.c_str());
        mvprintw(2, 0,"SAR Config: %s\t  Plane Angle: % 6.2f",SAR_Config.c_str(),Plane_Angle);


        mvprintw(4, 0,"==== Flags ====");
        mvprintw(5, 0,"Motorstop:\t%u  Flip_flag:\t  %u  Pos Ctrl:\t    %u  Cam_Est:\t  %u",Motorstop_Flag, flip_flag, Pos_Ctrl_Flag,isCamActive);
        mvprintw(6, 0,"Traj Active:\t%u  Impact_flag:\t  %u  Vel Ctrl:\t    %u ",Traj_Active_Flag,impact_flag,Vel_Ctrl_Flag);
        mvprintw(7, 0,"Policy_armed:\t%u  Tumble Detect: %u  Moment_Flag:   %u ",Policy_Armed_Flag,Tumble_Detection,Moment_Flag);
        mvprintw(8, 0,"Sticky_flag:\t%u  Tumbled:\t  %u  Slowdown_type: %u",Sticky_Flag,Tumbled_Flag,SLOWDOWN_TYPE);
        
        mvprintw(10, 0,"==== System States ====");
        mvprintw(11, 0,"Pos [m]:\t % 8.5f  % 8.5f  % 8.5f",Pose.position.x,Pose.position.y,Pose.position.z);
        mvprintw(12, 0,"Vel [m/s]:\t % 8.5f  % 8.5f  % 8.5f",Twist.linear.x,Twist.linear.y,Twist.linear.z);
        mvprintw(13, 0,"Omega [rad/s]:\t % 8.3f  % 8.3f  % 8.3f",Twist.angular.x,Twist.angular.y,Twist.angular.z);
        mvprintw(14, 0,"Eul [deg]:\t % 8.3f  % 8.3f  % 8.3f",Eul.x,Eul.y,Eul.z);
        mvprintw(15, 0,"Vel [mag,phi,alph]: % 8.3f % 8.3f",Vel_mag,Phi);

        mvprintw(17, 0,"==== Policy States ====");
        mvprintw(18, 0,"D_perp:  % 7.3f  V_perp: % 7.3f  V_tx:   % 7.3f",D_perp,V_perp,V_tx);
        mvprintw(19, 0,"Tau:     % 7.3f  Theta_x:     % 7.3f  Theta_y:     % 7.3f",Tau,Theta_x,Theta_y);
        mvprintw(20, 0,"Tau_est: % 7.3f  Theta_x_est: % 7.3f  Theta_y_est: % 7.3f",Tau_est,Theta_x_est,Theta_y_est);

        mvprintw(22, 0,"==== Policy: %s ====",POLICY_TYPE.c_str());
        if (strcmp(POLICY_TYPE.c_str(),"PARAM_OPTIM") == 0)
        {
            mvprintw(23, 0,"Tau_thr: % 7.3f \tMy: % 7.3f",Policy_Trg_Action,Policy_Flip_Action);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL_SB3") == 0)
        {
            mvprintw(23, 0,"Policy_Trg_Action: % 7.3f \tPolicy_Action: % 7.3f ",Policy_Trg_Action,Policy_Flip_Action);
        }
        else if (strcmp(POLICY_TYPE.c_str(),"DEEP_RL_ONBOARD") == 0)
        {
            mvprintw(23, 0,"Policy_Trg_Action: % 7.3f \tPolicy_Action: % 7.3f ",Policy_Trg_Action,Policy_Flip_Action);
        }

        mvprintw(25,0,"==== Flip Trigger Values ====");
        mvprintw(26,0,"Tau_tr:     % 7.3f \tPolicy_Trg_Action_tr:    % 7.3f ",Tau_tr,Policy_Trg_Action_tr);
        mvprintw(27,0,"\u03B8x_tr:     % 7.3f \tPolicy_Flip_Action_tr:  % 7.3f ",Theta_x_tr,Policy_Flip_Action_tr);
        mvprintw(28,0,"D_perp_tr:  % 7.3f ",D_perp_tr);

        mvprintw(30,0,"==== Setpoints ====");
        mvprintw(31,0,"x_d: % 7.3f  % 7.3f  % 7.3f",x_d.x,x_d.y,x_d.z);
        mvprintw(32,0,"v_d: % 7.3f  % 7.3f  % 7.3f",v_d.x,v_d.y,v_d.z);
        mvprintw(33,0,"a_d: % 7.3f  % 7.3f  % 7.3f",a_d.x,a_d.y,a_d.z);

        mvprintw(35,0,"==== Controller Actions ====");
        mvprintw(36,0,"FM [N/N*mm]: % 7.3f  % 7.3f  % 7.3f  % 7.3f",FM[0],FM[1],FM[2],FM[3]);
        mvprintw(37,0,"Motor Thrusts [g]: % 7.3f  % 7.3f  % 7.3f  % 7.3f",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
        mvprintw(38,0,"MS_PWM: %u  %u  %u  %u",MS_PWM[0],MS_PWM[1],MS_PWM[2],MS_PWM[3]);


        mvprintw(40,0,"=== Controller Gains ====");
        mvprintw(41,0,"Kp_P: % 7.3f  % 7.3f  % 7.3f ",P_kp_xy,P_kp_xy,P_kp_z);
        mvprintw(41,37,"Kp_R: % 7.3f  % 7.3f  % 7.3f ",R_kp_xy,R_kp_xy,R_kp_z);

        mvprintw(42,0,"Kd_P: % 7.3f  % 7.3f  % 7.3f ",P_kd_xy,P_kd_xy,P_kd_z);
        mvprintw(42,37,"Kd_R: % 7.3f  % 7.3f  % 7.3f ",R_kd_xy,R_kd_xy,R_kd_z);

        mvprintw(43,0,"Ki_P: % 7.3f  % 7.3f  % 7.3f ",P_ki_xy,P_ki_xy,P_ki_z);
        mvprintw(43,37,"Ki_R: % 7.3f  % 7.3f  % 7.3f ",R_ki_xy,R_ki_xy,R_ki_z);
        mvprintw(44,0,"======\n");


        // Refresh the screen with the updated buffer
        refresh();

        // Sleep for the desired delay time
        usleep(delay_time_us);
    }

    // Clean up and close the ncurses library
    endwin();
}