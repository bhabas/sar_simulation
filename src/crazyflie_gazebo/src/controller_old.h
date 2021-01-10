#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <Eigen/Dense>
//#include <boost/thread/thread.hpp>
#include <thread>

#include "readerwriterqueue.h"

typedef struct _StateFull {
    double data[18];
} StateFull;

typedef struct _MotorCommand {
    float data[4];
} MotorCommand;

class Controller
{
    public:
        //Controller();
        ~Controller()
        {
            isRunning = false;
            receiverThread_gazebo.join();
            senderThread_gazebo.join();
            receiverThread_RL.join();
            controllerThread.join();
            close(Ctrl_Mavlink_socket);
        }

        void Load();
        void recvThread_gazebo();
        void recvThread_RL();
        void controlThread();

    private:
        int Ctrl_Mavlink_socket;
        int Ctrl_Mavlink_socket_SNDBUF;
        int Ctrl_Mavlink_socket_RCVBUF;
        int Ctrl_Mavlink_socket_PORT;
        struct sockaddr_in addr_Ctrl_Mavlink;

        int Mavlink_PORT;
        struct sockaddr_in addr_Mavlink;
        socklen_t addr_Mavlink_len;
        
        
        int Ctrl_RL_socket;
        int Ctrl_RL_socket_SNDBUF;
        int Ctrl_RL_socket_RCVBUF;
        int Ctrl_RL_socket_Port;
        struct sockaddr_in addr_Ctrl_RL;

        int RL_PORT;
        struct sockaddr_in addr_RL;
        socklen_t addr_RL_len;

        bool isRunning;
        std::thread receiverThread_gazebo;
        std::thread senderThread_gazebo;
        std::thread receiverThread_RL;
        std::thread controllerThread;

        moodycamel::BlockingReaderWriterQueue<StateFull> queue_states;
        moodycamel::BlockingReaderWriterQueue<MotorCommand> queue_motorspeed;
        
        double control_cmd_recvd[5] = {555,0,0,0.0,0}; // Initial command, not sure but it's here
};


