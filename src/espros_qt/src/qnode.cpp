/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "espros_qt/qnode.hpp"

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv, Controller &controller) :
	init_argc(argc),
	init_argv(argv),
	controller(controller)
	{
		connect(&controller, &Controller::connected, this, &QNode::tcpConnected);
	  connect(&controller, &Controller::disconnected, this, &QNode::tcpDisconnected);
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"espros_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"espros_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	emit rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::tcpConnected() {
	std_msgs::String msg;
	msg.data = "QNode: tcp connected";
	chatter_publisher.publish(msg);

	std::cout << "QNode: tcp connected" << std::endl;
}

void QNode::tcpDisconnected() {
	std_msgs::String msg;
	msg.data = "QNode: tcp disconnected";
	chatter_publisher.publish(msg);

	std::cout << "QNode: tcp disconnected" << std::endl;
}
