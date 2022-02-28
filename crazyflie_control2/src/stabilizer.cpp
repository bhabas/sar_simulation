#include "stabilizer.h"
#include "controller_gtc.h"

void Controller::stabilizerLoop() // MAIN CONTROLLER LOOP
{
    ros::Rate rate(1000);
    tick = 1;
    safeModeEnable = false;
    // INITIATE CONTROLLER
    controllerGTCInit();
    // RUN STABILIZER LOOP
    while(ros::ok)
    {

        stateEstimator(&state, &sensorData, &control, tick); // Run state/sensor values through "Kalman filter"
        controllerGTC(&control, &setpoint, &sensorData, &state, tick);

        t = ros::Time::now();
    
        if (RATE_DO_EXECUTE(20, tick)) {
            Controller::consoleOuput();
        }

        Controller::checkSlowdown();
        


        MS_msg.MotorPWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};
        MS_PWM_Publisher.publish(MS_msg);

        // MISC INFO
        CTRL_msg.FM = {F_thrust,M.x*1.0e3,M.y*1.0e3,M.z*1.0e3};
        CTRL_msg.MS_PWM = {M1_pwm,M2_pwm,M3_pwm,M4_pwm};

        // NEURAL NETWORK INFO
        CTRL_msg.NN_policy = NN_policy;
        CTRL_msg.NN_flip = NN_flip;

        CTRL_msg.Tau = Tau;
        CTRL_msg.RREV = RREV;
        CTRL_msg.OFy = OFy;
        CTRL_msg.OFx = OFx;
        CTRL_msg.D_ceil = d_ceil;

        // FLIP INFO
        CTRL_msg.flip_flag = flip_flag;
        CTRL_msg.RREV_tr = RREV_tr;
        CTRL_msg.OFx_tr = OFx_tr;
        CTRL_msg.OFy_tr = OFy_tr;
        CTRL_msg.Tau_tr = Tau_tr;
        CTRL_msg.FM_flip = {F_thrust_flip,M_x_flip*1.0e3,M_y_flip*1.0e3,M_z_flip*1.0e3};

        CTRL_msg.NN_tr_flip = NN_tr_flip;
        CTRL_msg.NN_tr_policy = NN_tr_policy;

        // CTRL_msg.Pose_tr.header.stamp = t_flip;             

        CTRL_msg.Pose_tr.pose.position.x = statePos_tr.x;
        CTRL_msg.Pose_tr.pose.position.y = statePos_tr.y;
        CTRL_msg.Pose_tr.pose.position.z = statePos_tr.z;

        CTRL_msg.Pose_tr.pose.orientation.x = stateQuat_tr.x;
        CTRL_msg.Pose_tr.pose.orientation.y = stateQuat_tr.y;
        CTRL_msg.Pose_tr.pose.orientation.z = stateQuat_tr.z;
        CTRL_msg.Pose_tr.pose.orientation.w = stateQuat_tr.w;

        CTRL_msg.Twist_tr.linear.x = stateVel_tr.x;
        CTRL_msg.Twist_tr.linear.y = stateVel_tr.y;
        CTRL_msg.Twist_tr.linear.z = stateVel_tr.z;

        CTRL_msg.Twist_tr.angular.x = stateOmega_tr.x;
        CTRL_msg.Twist_tr.angular.y = stateOmega_tr.y;
        CTRL_msg.Twist_tr.angular.z = stateOmega_tr.z;
        CTRL_Publisher.publish(CTRL_msg);

        
        tick++;
        rate.sleep();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Stabilizer_Node");
    ros::NodeHandle nh;

    Controller SC = Controller(&nh);
    ros::spin();

    
    return 0;
}

