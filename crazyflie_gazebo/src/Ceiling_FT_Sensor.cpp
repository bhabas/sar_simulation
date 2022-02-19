#include "Ceiling_FT_Sensor.h"


void Ceiling_FT_Sensor::RLdata_Callback(const crazyflie_msgs::RLData::ConstPtr &msg)
{
    // RESET IMPACT DATA WHEN ROLLOUT IS RESET
    if(msg->reset_flag == true)
    {
        // RESET IMPACT FLAG AND VALUES
        _impact_flag = false;
        _ceiling_ft_x = 0.0;
        _ceiling_ft_y = 0.0;
        _ceiling_ft_z = 0.0;

        // RESET IMPACT VALUES WHENEVER RESET IS CALLED
        std::tie(_pos_impact.x,_pos_impact.y,_pos_impact.z) = std::make_tuple(0.0,0.0,0.0);
        std::tie(_vel_impact.x,_vel_impact.y,_vel_impact.z) = std::make_tuple(0.0,0.0,0.0);
        std::tie(_omega_impact.x,_omega_impact.y,_omega_impact.z) = std::make_tuple(0.0,0.0,0.0);
        std::tie(_quat_impact.x,_quat_impact.y,_quat_impact.z,_quat_impact.w) = std::make_tuple(0.0,0.0,0.0,1.0);

    }
}

void Ceiling_FT_Sensor::Surface_FT_Callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    // RECORD MAX FORCE EXPERIENCED
    if (msg->wrench.force.x > _ceiling_ft_x){
        _ceiling_ft_x = msg->wrench.force.x;
    }
    if (msg->wrench.force.y > _ceiling_ft_y){
        _ceiling_ft_y = msg->wrench.force.y;
    }
    if (msg->wrench.force.z > _ceiling_ft_z){
        _ceiling_ft_z = msg->wrench.force.z;
    }

    if (_ceiling_ft_z >= 0.1 && _impact_flag == false){ 
        // LOCK IN STATE DATA WHEN IMPACT DETECTED
        _impact_flag = true;

        // RECORD IMPACT STATE DATA FROM 2 DATAPOINTS BEHIND WHEN IMPACT FLAGGED
        _t_impact = ros::Time::now();
        _pos_impact = _pos_arr[2];
        _vel_impact = _vel_arr[2];
        _quat_impact = _quat_arr[2];
        _omega_impact = _omega_arr[2];
    }

  //PUBLISH THAT IMPACT OCCURED
  crazyflie_msgs::ImpactData impact_msg;

  impact_msg.impact_flag = _impact_flag;
  impact_msg.Header.stamp = _t_impact;

  // WRITE CURRENT MAX IMPACT FORCES TO MSG
  impact_msg.Force_impact.x = _ceiling_ft_x;
  impact_msg.Force_impact.y = _ceiling_ft_y;
  impact_msg.Force_impact.z = _ceiling_ft_z;

  // WRITE LAGGING IMPACT POSE TO MSG
  impact_msg.Pose_impact.position.x = _pos_impact.x;
  impact_msg.Pose_impact.position.y = _pos_impact.y;
  impact_msg.Pose_impact.position.z = _pos_impact.z;

  impact_msg.Pose_impact.orientation.x = _quat_impact.x;
  impact_msg.Pose_impact.orientation.y = _quat_impact.y;
  impact_msg.Pose_impact.orientation.z = _quat_impact.z;
  impact_msg.Pose_impact.orientation.w = _quat_impact.w;

  // WRITE LAGGING IMPACT TWIST TO MSG
  impact_msg.Twist_impact.linear.x = _vel_impact.x;
  impact_msg.Twist_impact.linear.y = _vel_impact.y;
  impact_msg.Twist_impact.linear.z = _vel_impact.z;
  
  impact_msg.Twist_impact.angular.x = _omega_impact.x;
  impact_msg.Twist_impact.angular.y = _omega_impact.y;
  impact_msg.Twist_impact.angular.z = _omega_impact.z;

  impactForce_Publisher.publish(impact_msg);
}

void Ceiling_FT_Sensor::vicon_stateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

    // SHIFT ALL ARRAY VALUES OVER BY ONE
    // Totally not the 'correct' way but it works for what I need right now

    _pos_arr[4] = _pos_arr[3];
    _pos_arr[3] = _pos_arr[2];
    _pos_arr[2] = _pos_arr[1];
    _pos_arr[1] = _pos_arr[0];
    _pos_arr[0] = msg->pose.pose.position;

    _vel_arr[4] = _vel_arr[3];
    _vel_arr[3] = _vel_arr[2];
    _vel_arr[2] = _vel_arr[1];
    _vel_arr[1] = _vel_arr[0];
    _vel_arr[0] = msg->twist.twist.linear;

    _omega_arr[4] = _omega_arr[3];
    _omega_arr[3] = _omega_arr[2];
    _omega_arr[2] = _omega_arr[1];
    _omega_arr[1] = _omega_arr[0];
    _omega_arr[0] = msg->twist.twist.angular;

    _quat_arr[4] = _quat_arr[3];
    _quat_arr[3] = _quat_arr[2];
    _quat_arr[2] = _quat_arr[1];
    _quat_arr[1] = _quat_arr[0];
    _quat_arr[0] = msg->pose.pose.orientation;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"Ceiling_FT_SensorNode");
    ros::NodeHandle nh;
    Ceiling_FT_Sensor Ceiling_FT_Init = Ceiling_FT_Sensor(&nh);
    ros::spin();
    return 1;
}