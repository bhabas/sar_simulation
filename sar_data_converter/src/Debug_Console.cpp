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
        mvprintw(5, 0,"SafeMode:   %u",SafeMode_Flag);
        mvprintw(6, 0,"MotorStop:  %u",MotorStop_Flag);
        mvprintw(7, 0,"Tumble:     %u",Tumbled_Flag);
        mvprintw(8, 0,"Tumbl_Det:  %u",TumbleDetect_Flag);

        mvprintw(5, 15,"Policy_Armed:    %u",Policy_Armed_Flag);
        mvprintw(6, 15,"Trg_Flag:        %u",Trg_Flag);
        mvprintw(7, 15,"Impct_Flag_Ext:  %u",Impact_Flag_Ext);
        mvprintw(8, 15,"Impct_Flag_OB:   %u",Impact_Flag_OB);

        mvprintw(5, 35,"Pos_Ctrl:       %u",Pos_Ctrl_Flag);
        mvprintw(6, 35,"Vel_Ctrl:       %u",Vel_Ctrl_Flag);
        mvprintw(7, 35,"Custom_PWM:     %u",CustomPWM_Flag);
        mvprintw(8, 35,"Custom_Thrust:  %u",CustomThrust_Flag);

        mvprintw(5, 55,"Ang_Accel:   %u",AngAccel_Flag);
        mvprintw(6, 55,"Cam_Active:  %u",CamActive_Flag);
        mvprintw(7, 55,"Sticky_Flag: %u",Sticky_Flag);


        mvprintw(10, 0,"============== World States =============");
        mvprintw(11, 0,"r_B_O [m]:           % 6.2f % 6.2f % 6.2f",Pose_B_O.position.x,Pose_B_O.position.y,Pose_B_O.position.z);
        mvprintw(12, 0,"V_B_O [m/s]:         % 6.2f % 6.2f % 6.2f",Twist_B_O.linear.x,Twist_B_O.linear.y,Twist_B_O.linear.z);
        mvprintw(13, 0,"a_B_O [m/s^2]:       % 6.2f % 6.2f % 6.2f",Accel_B_O.linear.x,Accel_B_O.linear.y,Accel_B_O.linear.z);
        mvprintw(14, 0,"Omega_B_O [rad/s]:   % 6.2f % 6.2f % 6.2f",Twist_B_O.angular.x,Twist_B_O.angular.y,Twist_B_O.angular.z);
        mvprintw(15, 0,"AngAcc_B_O [rad/s^2]:% 6.2f % 6.2f % 6.2f",Accel_B_O.angular.x,Accel_B_O.angular.y,Accel_B_O.angular.z);
        mvprintw(16, 0,"V_B_O:  (% 5.2f % 6.2f)",Vel_mag_B_O,Vel_angle_B_O);

        mvprintw(10, 44,"============== Rel. States ==============");
        mvprintw(11, 44,"r_P_O [m]:           % 6.2f % 6.2f % 6.2f",Plane_Pos.x,Plane_Pos.y,Plane_Pos.z);
        mvprintw(12, 44,"r_P_B [m]:           % 6.2f % 6.2f % 6.2f",Pose_P_B.position.x,Pose_P_B.position.y,Pose_P_B.position.z);
        mvprintw(13, 44,"V_B_P [m/s]:         % 6.2f % 6.2f % 6.2f",Twist_B_P.linear.x,Twist_B_P.linear.y,Twist_B_P.linear.z);
        mvprintw(14, 44,"Eul_B_O [deg]:       % 6.2f % 6.2f % 6.2f",Eul_B_O.x,Eul_B_O.y,Eul_B_O.z);
        mvprintw(15, 44,"Eul_P_B [deg]:       % 6.2f % 6.2f % 6.2f",Eul_P_B.x,Eul_P_B.y,Eul_P_B.z);
        mvprintw(16, 44,"V_B_P:  (% 5.2f % 6.2f)",Vel_mag_B_P,Vel_angle_B_P);


        mvprintw(18,  0,"====== Policy: %s ======",POLICY_TYPE.c_str());
        mvprintw(19, 0,"Tau:    % 6.3f",Tau);
        mvprintw(20, 0,"Tau_CR: % 6.3f",Tau_CR);

        mvprintw(19, 20,"Theta_x: % 6.3f",Theta_x);
        mvprintw(20, 20,"D_perp:  % 6.3f",D_perp);

        mvprintw(19, 42,"Trg_Act: % 6.3f",Policy_Trg_Action);
        mvprintw(20, 42,"Rot_Act: % 6.3f",Policy_Rot_Action);

        mvprintw(22, 0,"============= Trigger States ============");
        mvprintw(23, 0,"Tau_trg:    % 6.3f",Tau_trg);
        mvprintw(24, 0,"Tau_CR_trg: % 6.3f",Tau_CR_trg);

        mvprintw(23, 20,"Theta_x_trg: % 6.3f",Theta_x_trg);
        mvprintw(24, 20,"D_perp_trg:  % 6.3f",D_perp_trg);

        mvprintw(23, 42,"Trg_Act_trg: % 6.3f",Policy_Trg_Action_trg);
        mvprintw(24, 42,"Rot_Act_trg: % 6.3f",Policy_Rot_Action_trg);

        mvprintw(25, 0,"V_B_O_trg:  (% 5.2f % 6.2f)",Vel_mag_B_O_trg,Vel_angle_B_O_trg);
        mvprintw(25, 0,"Eul_P_B_trg [deg]:   % 6.2f",Eul_P_B_trg.y);
        mvprintw(25, 42,"V_B_P_trg: (% 5.2f % 6.2f)",Vel_mag_B_P_trg,Vel_angle_B_P_trg);

        mvprintw(27, 0,"============= Impact States ============");

        mvprintw(28, 0,"Body Contact: %u",BodyContact_Flag);
        mvprintw(29, 0,"Pad Connect:  %u",Pad_Connections);
        
        mvprintw(28, 17,"Fore Contact: %u",ForelegContact_Flag);
        mvprintw(29, 17,"Hind Contact: %u",HindlegContact_Flag);

        mvprintw(28, 34,"Phi_B_O_impact: % 6.3f",Eul_B_O_impact_Ext.y);
        mvprintw(29, 34,"Phi_P_B_impact: % 6.3f",Eul_P_B_impact_Ext.y);




        mvprintw(31,0,"==== Setpoints ====");
        mvprintw(32,0,"x_d: % 7.3f  % 7.3f  % 7.3f",x_d.x,x_d.y,x_d.z);
        mvprintw(33,0,"v_d: % 7.3f  % 7.3f  % 7.3f",v_d.x,v_d.y,v_d.z);
        mvprintw(34,0,"a_d: % 7.3f  % 7.3f  % 7.3f",a_d.x,a_d.y,a_d.z);

        mvprintw(36,0,"==== Controller Actions ====");
        mvprintw(37,0,"FM [N/N*mm]:       % 7.3f  % 7.3f  % 7.3f  % 7.3f",FM[0],FM[1],FM[2],FM[3]);
        mvprintw(38,0,"Motor Thrusts [g]: % 7.3f  % 7.3f  % 7.3f  % 7.3f",MotorThrusts[0],MotorThrusts[1],MotorThrusts[2],MotorThrusts[3]);
        mvprintw(39,0,"MS_Cmd: %u  %u  %u  %u",Motor_CMD[0],Motor_CMD[1],Motor_CMD[2],Motor_CMD[3]);


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