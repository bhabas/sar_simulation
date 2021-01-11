#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include "controller.h"



using namespace Eigen;
using namespace std;

void Controller::Load()
{
    cout << setprecision(3);
    cout << fixed;
    _isRunning = true;

    // INIT FIRST CONTROLLER SOCKET (COMMUNICATES W/ MAVLINK PORT:18080)
    Ctrl_Mavlink_socket = socket(AF_INET, SOCK_DGRAM, 0); // DGRAM is for UDP communication (Send data but don't care if it's recieved)
    Ctrl_Mavlink_socket_SNDBUF = 16;    // 4 floats [16 bytes] for Motorspeeds       
    Ctrl_Mavlink_socket_RCVBUF = 144;   // 18 doubles [144 bytes] for State array
    Ctrl_Mavlink_socket_PORT = 18070;   // Port for this socket

    // SET EXPECTED BUFFER SIZES
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_SNDBUF, &Ctrl_Mavlink_socket_SNDBUF, sizeof(Ctrl_Mavlink_socket_SNDBUF))<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Setting SNDBUF"<<endl;
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_RCVBUF, &Ctrl_Mavlink_socket_RCVBUF, sizeof(Ctrl_Mavlink_socket_RCVBUF))<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Setting RCVBUF"<<endl;
    int enable = 1; // Fix for error if socket hasn't close correctly when restarting program
    if (setsockopt(Ctrl_Mavlink_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        cout <<"help me"<< endl;

    // SET SOCKET SETTINGS
    memset(&addr_Ctrl_Mavlink, 0, sizeof(addr_Ctrl_Mavlink));
    addr_Ctrl_Mavlink.sin_family = AF_INET;
    addr_Ctrl_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("0.0.0.0");
    addr_Ctrl_Mavlink.sin_port = htons(Ctrl_Mavlink_socket_PORT);

    // BIND ADDRESS TO CONTROLLER SOCKET (PORT:18070)
    if (bind(Ctrl_Mavlink_socket, (struct sockaddr*)&addr_Ctrl_Mavlink, sizeof(addr_Ctrl_Mavlink)) < 0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Binding address to socket"<<endl;
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Binding address to socket"<<endl; 


    


    // INIT SECOND CONTROLLER SOCKET (COMMUNICATES W/ RL PORT:18050)
    Ctrl_RL_socket = socket(AF_INET, SOCK_DGRAM, 0);
    Ctrl_RL_socket_SNDBUF = 144; // 18 doubles [144 bytes] for State array
    Ctrl_RL_socket_RCVBUF = 40;  // 5 doubles [8 bytes] for Controller Commands
    Ctrl_RL_socket_Port = 18060; // Port for this socket

    // SET EXPECTED BUFFER SIZES
    if (setsockopt(Ctrl_RL_socket, SOL_SOCKET, SO_SNDBUF, &Ctrl_RL_socket_SNDBUF, sizeof(Ctrl_RL_socket_SNDBUF))<0)
        cout<<"[FAILED] Ctrl_RL_socket: Setting SNDBUF"<<endl;
    if (setsockopt(Ctrl_RL_socket, SOL_SOCKET, SO_RCVBUF, &Ctrl_RL_socket_RCVBUF, sizeof(Ctrl_RL_socket_RCVBUF))<0)
        cout<<"[FAILED] Ctrl_RL_socket: Setting RCVBUF"<<endl;
    // Fix for error if socket hasn't close correctly when restarting program
    if (setsockopt(Ctrl_RL_socket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        cout <<"help me"<< endl;
    

    // SET SOCKET SETTINGS
    memset(&addr_Ctrl_RL, 0, sizeof(addr_Ctrl_RL)); // Not sure what this does
    addr_Ctrl_RL.sin_family = AF_INET; // IPv4 Format
    addr_Ctrl_RL.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("127.0.0.1");
    addr_Ctrl_RL.sin_port = htons(Ctrl_RL_socket_Port); // RL Port number
    
    
    // BIND ADDRESS TO SECOND CONTROLLER SOCKET (PORT:18060)
    if (bind(Ctrl_RL_socket, (struct sockaddr*)&addr_Ctrl_RL, sizeof(addr_Ctrl_RL))<0)
        cout<<"[FAILED] Ctrl_RL_socket: Binding address to socket"<<endl;
    else
        cout<<"[SUCCESS] Ctrl_RL_socket: Binding address to socket"<<endl; 

    // INIT ADDRESS FOR MAVLINK SOCKET (PORT: 18080)
    Mavlink_PORT = 18080;
    memset(&addr_Mavlink, 0, sizeof(addr_Mavlink));
    addr_Mavlink.sin_family = AF_INET;
    addr_Mavlink.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_Mavlink.sin_port = htons(Mavlink_PORT);
    addr_Mavlink_len = sizeof(addr_Mavlink);

    // INIT ADDRESS FOR RL SOCKET (PORT:18050)
    RL_PORT = 18050;
    memset(&addr_RL, 0, sizeof(addr_RL));
    addr_RL.sin_family = AF_INET;
    addr_RL.sin_addr.s_addr = htonl(INADDR_ANY);
    addr_RL.sin_port = htons(RL_PORT);
    addr_RL_len = sizeof(addr_RL);

    // MOTORSPEED TO MAVLINK TEST (VISUALLY CONFIRMS THINGS ARE WORKING IN SIM)
    float msg[4] = {1900,1900,1900,1900};
    int msg_len = 0;
    for(int k=0; k<2; k++)
        // To Gazebo socket, send msg of len(msg)
        msg_len = sendto(Ctrl_Mavlink_socket, msg, sizeof(msg),0, (struct sockaddr*)&addr_Mavlink, sizeof(addr_Mavlink));
    if(msg_len<0)
        cout<<"[FAILED] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Threads will mutual lock!"<<endl; // Not sure what mutual lock means
    else
        cout<<"[SUCCESS] Ctrl_Mavlink_socket: Sending test motor speeds to Mavlink. Avoiding mutual locking between threads!"<<endl;

    // START COMMUNICATION THREADS
    receiverThread_RL = std::thread(&Controller::recvThread_RL, this);
    controllerThread = std::thread(&Controller::controlThread, this);


}

void Controller::recvThread_RL()
{
    float motorspeed_fake[4] = {0,0,0,0};

    while(_isRunning)
    {
        //cout<<"[recvThread_RL] Receiving command from RL"<<endl;
        int len = recvfrom(Ctrl_RL_socket, control_cmd_recvd, sizeof(control_cmd_recvd),0, (struct sockaddr*)&addr_RL, &addr_RL_len);


        if(control_cmd_recvd[0]>10) // If header is 11 then enable sticky
        {
            motorspeed_fake[0] = -control_cmd_recvd[0];
            motorspeed_fake[1] = control_cmd_recvd[1];
            //cout<<"Send sticky command command: "<< motorspeed_fake[0]<<", "<<motorspeed_fake[1]<<endl;
            sendto(Ctrl_Mavlink_socket, motorspeed_fake, sizeof(motorspeed_fake),0, (struct sockaddr*)&addr_Mavlink, addr_Mavlink_len);

            
        }
    }
}

void Controller::callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg){

    // SIMPLIFY STATE VALUES FROM TOPIC
    float _t = msg->header.stamp.toSec();
    const geometry_msgs::Point position = msg->global_pose.position; // Follow names from message details - "rqt -s rqt_msg" 
    const geometry_msgs::Quaternion quaternion = msg->global_pose.orientation;
    const geometry_msgs::Vector3 velocity = msg->global_twist.linear;
    const geometry_msgs::Vector3 omega = msg->global_twist.angular;

    // SET STATE VALUES INTO CLASS STATE VARIABLES
    _pos << position.x, position.y, position.z;
    _vel << velocity.x, velocity.y, velocity.z;
    _quat << quaternion.w, quaternion.x, quaternion.y, quaternion.z, 
    _omega << omega.x, omega.y, omega.z;


    // std::cout << _vel.transpose() << std::endl;

}

void Controller::controlThread()
{
    // =========== Controller Explanation =========== //
    // https://www.youtube.com/watch?v=A27knigjGS4&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=46
    // Derived from DOI's: 10.1002/asjc.567 (T. Lee) & 10.13140/RG.2.2.12876.92803/1 (M. Fernando)
    typedef Matrix<double, 3, 3, RowMajor> RowMatrix3d; 


    MotorCommand motorspeed_structure;

    float motorspeed[4];
    double state_full[18];
    

    int type; // Command type {1:Pos, 2:Vel, 3:Att, 4:Omega, 5:Stop}
    double ctrl_flag; // On/Off switch for controller
    double control_cmd[5];
    Vector3d control_vals;

    
    

}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);
    controller.Load();
    ros::spin();
}