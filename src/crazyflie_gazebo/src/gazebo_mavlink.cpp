#include <iostream>
#include <gazebo_mavlink.h>
#include <ignition/math.hh>

namespace gazebo{

void GazeboMavlink::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) 
{
    gzmsg<<"!!!!! Entering GazeboMavlink::Load !!!!!\n";
    model_ = _model;

    if (_sdf->HasElement("robotNamespace"))
    {
        namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
        // std::cout<<"!!!!! namespace_ = !!!!! "<<namespace_<<std::endl;
    }
    else
        gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
    node_handle_ = transport::NodePtr(new transport::Node());
    node_handle_->Init(namespace_);

    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboMavlink::OnUpdate, this, _1));

    getSdfParam<std::string>(_sdf, "motorSpeedCommandPubTopic", motor_velocity_reference_pub_topic_, motor_velocity_reference_pub_topic_);
    motor_velocity_reference_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + motor_velocity_reference_pub_topic_, 1);
    getSdfParam<std::string>(_sdf, "stickyEnablePubTopic", sticky_enable_pub_topic_, sticky_enable_pub_topic_);
    sticky_enable_pub_ = node_handle_->Advertise<mav_msgs::msgs::CommandMotorSpeed>("~/" + model_->GetName() + sticky_enable_pub_topic_, 1);

    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
    gzmsg<<"!!!!! link_name_ ="<<link_name_<<"\n";
    link_ = model_->GetLink(link_name_);
    if (link_ == NULL)
        gzerr<<"[gazebo_mavlink] Couldn't find specified link " << link_name_ << std::endl;

    std::string mavlink_addr_str = _sdf->GetElement("mavlink_addr")->Get<std::string>();
    mavlink_port_ = _sdf->GetElement("mavlink_port")->Get<int>();
    if ((fd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        gzerr << "Socket creating failed, aborting..." << std::endl;
        abort();
    }
    fd_SNDBUF_ = 112;        // 112 bytes is 14 double
    fd_RCVBUF_ = 16;         // 16 bytes is 4 float
    if (setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &fd_SNDBUF_, sizeof(fd_SNDBUF_))<0)
        std::cout<<"fd_ setting SNDBUF failed"<<std::endl;
    if (setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &fd_RCVBUF_, sizeof(fd_RCVBUF_))<0)
        std::cout<<"fd_ setting RCVBUF failed"<<std::endl;
    memset((char *)&sockaddr_local_, 0, sizeof(sockaddr_local_));
    sockaddr_local_.sin_family = AF_INET;
    if (mavlink_addr_str == "INADDR_ANY")
        sockaddr_local_.sin_addr.s_addr = htonl(INADDR_ANY);
    else if (inet_addr(mavlink_addr_str.c_str()) == INADDR_NONE)
    {
        gzerr << "Invalid mavlinnk address, aborting..." << std::endl;
        abort();
    }
    else
        sockaddr_local_.sin_addr.s_addr = inet_addr(mavlink_addr_str.c_str());
    sockaddr_local_.sin_port = htons(mavlink_port_);
    /*sockaddr_local_.sin_family = AF_INET;
    sockaddr_local_.sin_addr.s_addr = htonl(INADDR_ANY);
    sockaddr_local_.sin_port = htons(18080);*/
    if (bind(fd_, (struct sockaddr *)&sockaddr_local_, sizeof(sockaddr_local_)) < 0)
    {
        gzerr << "Socket binding failed, aborting..." << std::endl;
        abort();
    }

    memset(&sockaddr_remote_, 0, sizeof(sockaddr_remote_));
    
    isPluginOn_ = true;
    receiverThread = std::thread(&GazeboMavlink::recvThread, this);
    senderThread = std::thread(&GazeboMavlink::sendThread , this);

}

// This gets called by the world update start event.
void GazeboMavlink::OnUpdate(const common::UpdateInfo& _info) 
{
    sampling_time_ = _info.simTime.Double() - prev_sim_time_;
    prev_sim_time_ = _info.simTime.Double();
    // gzmsg<<"sampling_time = "<<sampling_time_<<std::endl;
    // std::cout<<"Simulation time = "<<prev_sim_time_<<std::endl;

    ignition::math::Pose3d pose;
    ignition::math::Vector3d vel_linear, vel_angular;

    pose = link_->WorldPose();
    vel_linear = link_->WorldLinearVel();
    vel_angular = link_->WorldAngularVel();

    //double state_full[13];

    double position[] = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
    // pose.Rot().Normalize();
    double orientation_q[] = {pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z()};
    double vel[] = {vel_linear.X(), vel_linear.Y(), vel_linear.Z()};
    double omega[] = {vel_angular.X(), vel_angular.Y(), vel_angular.Z()};

    std::memcpy(state_full, position, sizeof(position));
    std::memcpy(state_full+3, orientation_q, sizeof(orientation_q));
    std::memcpy(state_full+7, vel, sizeof(vel));
    std::memcpy(state_full+10, omega, sizeof(omega));
    state_full[13] = prev_sim_time_;

    //std::cout<<"Altitude: "<<position[2]<<std::endl;

    //if (fmod(prev_sim_time_,1.0) == 0)      // fmod is the operator % for double
    {
        //gzmsg<<"======================="<<std::endl;
        //gzmsg<<"sampling_time = "<<sampling_time_<<std::endl;
        /*std::cout<<"Full State:";
        for(int k=0;k<13;k++)
            std::cout<<state_full[k]<<", ";
        std::cout<<"\n";
        gzmsg<<"position = ["<<position[0]<<", "<<position[1]<<", "<<position[2]<<"]\n";
        gzmsg<<"orientation_q = ["<<orientation_q[0]<<", "<<orientation_q[1]<<", "<<orientation_q[2]<<", "<<orientation_q[3]<<"]\n";
        gzmsg<<"vel = ["<<vel[0]<<", "<<vel[1]<<", "<<vel[2]<<"]\n";
        gzmsg<<"omega = ["<<omega[0]<<", "<<omega[1]<<", "<<omega[2]<<"]\n";*/
      
    }
}

void GazeboMavlink::recvThread()
{
    float motor_speed[4];
    mav_msgs::msgs::CommandMotorSpeed turning_velocities_msg;
    
    while(isPluginOn_)
    {
        int len = recvfrom(fd_, motor_speed, sizeof(motor_speed),0, 
        (struct sockaddr*)&sockaddr_remote_, &sockaddr_remote_len_);
        if (len>0)
        {
            //std::cout<<"Received "<<len<<" byte motor speed [";
            //std::cout<<motor_speed[0]<<", "<<motor_speed[1]<<", "<<motor_speed[2]<<", "<<motor_speed[3]<<"]"<<std::endl;
            
            turning_velocities_msg.clear_motor_speed();
            for(int k=0; k<4;k++)
            {
                turning_velocities_msg.add_motor_speed(motor_speed[k]);
            }
            if(motor_speed[0]>=0)
                motor_velocity_reference_pub_->Publish(turning_velocities_msg);
            else
                sticky_enable_pub_->Publish(turning_velocities_msg);
        }      
    }
    
}

void GazeboMavlink::sendThread()
{
    double states[14];
    
    while(isPluginOn_)
    {
        usleep(1000);       // micro second delay, prevent thread from running too fast
        std::memcpy(states, state_full, sizeof(states));
        int len=sendto(fd_, states, sizeof(states),0, (struct sockaddr*)&sockaddr_remote_, sockaddr_remote_len_);
        /*if (len>0)
            std::cout<<"sendto succeed"<<std::endl;
        else
            std::cout<<"sendto failed"<<std::endl;*/
    }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboMavlink);
}