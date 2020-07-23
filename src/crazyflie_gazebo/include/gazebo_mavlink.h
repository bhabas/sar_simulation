#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>

#include "common.h"     // provides getSdfParam, FirstOrderFilter ...

#include "CommandMotorSpeed.pb.h"

#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

namespace gazebo {

class GazeboMavlink: public ModelPlugin
{
    public: 
        /*GazeboMavlink() : ModelPlugin()
        {
            std::cout<<"!!!!! GazeboMavlink Loaded !!!!!"<<std::endl;
        }*/
        ~GazeboMavlink()
        {
            isPluginOn_ = false;
            senderThread.join();
		    receiverThread.join();
            close(fd_); 
        }

    protected:
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void OnUpdate(const common::UpdateInfo&  /*_info*/);

    private:
        physics::ModelPtr model_;
        physics::LinkPtr link_;

        transport::NodePtr node_handle_;
        transport::PublisherPtr motor_velocity_reference_pub_;
        transport::PublisherPtr sticky_enable_pub_;

        std::string namespace_;
        std::string link_name_;
        std::string motor_velocity_reference_pub_topic_;
        std::string sticky_enable_pub_topic_;

        event::ConnectionPtr updateConnection_;

        double prev_sim_time_;
        double sampling_time_;
        
        int fd_;
        int fd_SNDBUF_;
        int fd_RCVBUF_;
        int mavlink_port_;
        struct sockaddr_in sockaddr_local_;
        struct sockaddr_in sockaddr_remote_;
        socklen_t sockaddr_remote_len_;

        bool isPluginOn_;
        std::thread receiverThread;
		void recvThread();
        std::thread senderThread;
		void sendThread();

        double state_full[14];
};

}