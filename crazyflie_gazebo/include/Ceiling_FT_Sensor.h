#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "crazyflie_msgs/RLData.h"
#include "crazyflie_msgs/ImpactData.h"
#include "nav_msgs/Odometry.h"


class Ceiling_FT_Sensor
{
    public:

        Ceiling_FT_Sensor(ros::NodeHandle *nh)
        {
            impactForce_Publisher = nh->advertise<crazyflie_msgs::ImpactData>("CF_DC/ImpactData", 1);
            gloabalState_Subscriber = nh->subscribe("/env/global_state_data",1,&Ceiling_FT_Sensor::vicon_stateCallback,this,ros::TransportHints().tcpNoDelay());
            RLdata_Subscriber = nh->subscribe("/RL/data",5,&Ceiling_FT_Sensor::RLdata_Callback,this);  
            Surface_FT_Subscriber = nh->subscribe("/Gazebo/Ceiling_FT",5,&Ceiling_FT_Sensor::Surface_FT_Callback,this,ros::TransportHints().tcpNoDelay());  
        }

        ros::Publisher impactForce_Publisher;
        ros::Subscriber gloabalState_Subscriber;
        ros::Subscriber RLdata_Subscriber;
        ros::Subscriber Surface_FT_Subscriber;

        void vicon_stateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void RLdata_Callback(const crazyflie_msgs::RLData::ConstPtr &msg);
        void Surface_FT_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

    private:

        double _ceiling_ft_x = 0.0; // Max impact force in X-direction [N]
        double _ceiling_ft_y = 0.0; // Max impact force in Y-direction [N]
        double _ceiling_ft_z = 0.0; // Max impact force in Z-direction [N]
        bool _impact_flag = false;


        geometry_msgs::Point _pos;        // Current position [m]
        geometry_msgs::Vector3 _vel;      // Current velocity [m]
        geometry_msgs::Quaternion _quat;  // Current attitude [rad] (quat form)
        geometry_msgs::Vector3 _omega;    // Current angular velocity [rad/s]


        geometry_msgs::Vector3 _vel_prev;      // Current velocity [m]

        ros::Time _t_impact;                      // Impact time [s]
        geometry_msgs::Point _pos_impact;         // Impact position [m]
        geometry_msgs::Vector3 _vel_impact;       // Impact velocity [m]
        geometry_msgs::Quaternion _quat_impact;   // Impact attitude [rad] (quat form)
        geometry_msgs::Vector3 _omega_impact;     // Impact angular velocity [rad/s]



        const int arr_len = 5;
        geometry_msgs::Point _pos_arr[5];
        geometry_msgs::Vector3 _vel_arr[5];
        geometry_msgs::Quaternion _quat_arr[5];
        geometry_msgs::Vector3 _omega_arr[5];


};
