#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

int main(int argc, char** argv)
{
  if (!ros::isInitialized()) {
    ros::init(argc, argv, "talker");
  }
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = std::string("Hello World");

    chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

