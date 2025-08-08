#include <ros/ros.h>
#include <std_msgs/String.h>

void callback_receive_data(const std_msgs::String& msg)
{
  ROS_INFO("Message received : %s", msg.data.c_str());
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "cplusplus_subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/pub", 
    1000, callback_receive_data );
  ros::spin();
}
