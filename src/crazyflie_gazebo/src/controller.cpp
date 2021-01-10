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
    isRunning = true;

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





}


int main(int argc, char **argv)
{
    ros::init(argc, argv,"controller_node");
    ros::NodeHandle nh;
    Controller controller = Controller(&nh);
    controller.Load();
    ros::spin();
}