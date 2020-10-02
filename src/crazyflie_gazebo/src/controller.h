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
            receiverThread_rl.join();
            controllerThread_.join();
            close(fd_gazebo);
        }

        void Load(int gazebo_port_number);
        void recvThread_gazebo();
        void sendThread_gazebo();
        void recvThread_rl();
        void controlThread();

    private:
        int fd_gazebo;
        int fd_gazebo_SNDBUF;
        int fd_gazebo_RCVBUF;
        int port_number_gazebo;
        struct sockaddr_in sockaddr_local_gazebo;
        struct sockaddr_in sockaddr_remote_gazebo;
        socklen_t sockaddr_remote_gazebo_len;
        int fd_rl;
        int fd_rl_SNDBUF;
        int fd_rl_RCVBUF;
        struct sockaddr_in sockaddr_local_rl;
        struct sockaddr_in sockaddr_remote_rl;
        socklen_t sockaddr_remote_rl_len;

        bool isRunning;
        std::thread receiverThread_gazebo;
        std::thread senderThread_gazebo;
        std::thread receiverThread_rl;
        std::thread controllerThread_;

        moodycamel::BlockingReaderWriterQueue<StateFull> queue_states;
        moodycamel::BlockingReaderWriterQueue<MotorCommand> queue_motorspeed;
        
        double control_cmd[5] = {2,0,0,0.0,0};
};
