#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <gazebo_communication_pkg/GlobalState.h>



// Reference: https://roboticsbackend.com/oop-with-ros-in-cpp/

class NumberCounter {

    private:
    int counter;
    ros::Publisher pub;
    ros::Subscriber number_subscriber;


    public:
    void callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg);
    NumberCounter(ros::NodeHandle *nh) {
        counter = 0;
        
        pub = nh->advertise<std_msgs::Int64>("/number_count",10);
        number_subscriber = nh->subscribe("/global_state",1000,
            &NumberCounter::callback_number,this);

    }


};

void NumberCounter::callback_number(const gazebo_communication_pkg::GlobalState::ConstPtr &msg){

    std::cout << "test" << std::endl;
}



int main(int argc, char **argv)
{
    std::cout << "test" << std::endl;
    ros::init(argc, argv,"number_counter");
    ros::NodeHandle nh;
    NumberCounter nc = NumberCounter(&nh);

    ros::spin();
}