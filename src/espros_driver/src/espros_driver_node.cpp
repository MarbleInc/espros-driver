#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//-------- Espros Headers ------------------//
/*
#include "controller.h"
#include "settings.h"
#include "interface.h"
*/



int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "espros_driver_node");
  ros::NodeHandle n;
/*
  // init espros
  Interface interface;
  Settings settings;
  Controller controller(settings, interface);
*/
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
