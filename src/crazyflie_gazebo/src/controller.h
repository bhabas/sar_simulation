#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
//#include <boost/thread/thread.hpp>
#include <thread>

#include "readerwriterqueue.h"

typedef struct _StateFull {
    double data[14];
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
            close(socket_Gazebo);
        }

        void Load(int gazebo_port_number);
        void recvThread_gazebo();
        void sendThread_gazebo();
        void recvThread_RL();
        void controlThread();

    private:
        int socket_Gazebo;
        int fd_gazebo_SNDBUF;
        int fd_gazebo_RCVBUF;
        int port_number_gazebo_;
        struct sockaddr_in sockaddr_local_Gazebo;
        struct sockaddr_in sockaddr_remote_Gazebo;
        socklen_t sockaddr_remote_gazebo_len;
        
        int fd_RL;
        int fd_RL_SNDBUF;
        int fd_RL_RCVBUF;
        struct sockaddr_in sockaddr_local_RL;
        struct sockaddr_in sockaddr_remote_rl;
        socklen_t sockaddr_remote_rl_len;

        bool isRunning;
        std::thread receiverThread_gazebo;
        std::thread senderThread_gazebo;
        std::thread receiverThread_RL;
        std::thread controllerThread;

        moodycamel::BlockingReaderWriterQueue<StateFull> queue_states;
        moodycamel::BlockingReaderWriterQueue<MotorCommand> queue_motorspeed;
        
        double control_cmd_recvd[5] = {555,0,0,0.0,0}; // Initial command, not sure but it's here
};
