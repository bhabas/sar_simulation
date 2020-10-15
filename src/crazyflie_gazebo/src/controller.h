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
            isRunning_ = false;
            receiverThread_gazebo_.join();
            senderThread_gazebo_.join();
            receiverThread_rl_.join();
            controllerThread_.join();
            close(fd_gazebo_);
        }

        void Load(int gazebo_port_number);
        void recvThread_gazebo();
        void sendThread_gazebo();
        void recvThread_rl();
        void controlThread();

    private:
        int fd_gazebo_;
        int fd_gazebo_SNDBUF_;
        int fd_gazebo_RCVBUF_;
        int port_number_gazebo_;
        struct sockaddr_in sockaddr_local_gazebo_;
        struct sockaddr_in sockaddr_remote_gazebo_;
        socklen_t sockaddr_remote_gazebo_len_;
        int fd_rl_;
        int fd_rl_SNDBUF_;
        int fd_rl_RCVBUF_;
        struct sockaddr_in sockaddr_local_rl_;
        struct sockaddr_in sockaddr_remote_rl_;
        socklen_t sockaddr_remote_rl_len_;

        bool isRunning_;
        std::thread receiverThread_gazebo_;
        std::thread senderThread_gazebo_;
        std::thread receiverThread_rl_;
        std::thread controllerThread_;

        moodycamel::BlockingReaderWriterQueue<StateFull> queue_states_;
        moodycamel::BlockingReaderWriterQueue<MotorCommand> queue_motorspeed_;
        
        double control_cmd_recvd[5] = {555,0,0,0.0,0};
};
