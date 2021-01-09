#include <cpp_python/InfoData.h>
#include <cpp_python/InfoDataArray.h>
#include <ros/ros.h>

void infodataCallback(const cpp_python::InfoDataArray::ConstPtr &msg){

    ROS_INFO("%d",msg->infos[0].num);
    ROS_INFO("%s",msg->infos[0].color.c_str());
    ROS_INFO("%s",msg->infos[0].name.c_str());

}

int main(int argc, char **argv){
    ros::init(argc,argv,"infodata_subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/infodata",1000,infodataCallback);

    ros::spin();
    return 0;
}