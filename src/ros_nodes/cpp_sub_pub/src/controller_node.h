#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <gazebo_communication_pkg/GlobalState.h>
#include "cpp_sub_pub/my_msg.h"


class Controller 
{

    public:
    Controller(ros::NodeHandle *nh) {
        counter = 0;
        
        pub = nh->advertise<cpp_sub_pub::my_msg>("/ctrlData",10);
        number_subscriber = nh->subscribe("/global_state",1000,
            &Controller::callback_number,this);
        controllerThread = std::thread(&Controller::controlThread, this);

    }
    float alpha;


    void callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg);
    void controlThread();

    private:
      int counter;
      ros::Publisher pub;
      ros::Subscriber number_subscriber;

      std::thread controllerThread;

};