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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
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
		connect(&controller, &Controller::receivedMeasurementData, this, &QNode::showDistance);
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
	distance_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance", 100);
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
	distance_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance", 100);
	start();
	return true;
}

void QNode::setSettings(const Settings *settings)
{
  this->settings = (Settings *)settings;
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

void QNode::showDistance(const char *pData, DataHeader &dataHeader)
{
	imageColorizerDistance.setRange(0, settings->getRange());

	sensor_msgs::Image img;

	img.header.stamp = ros::Time::now();
	img.header.frame_id = "1";

	img.encoding = sensor_msgs::image_encodings::RGB8;//MONO16;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 3;// * 2;

	img.data.resize(img.step * img.height);

  int index = 0;
  for (int y = 0; y < dataHeader.height; y++)
  {
    for (int x = 0; x < dataHeader.width; x++)
    {
      //First get the values of the distance
      uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
      uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

			//Combint to a value
      unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;
			QColor color = imageColorizerDistance.getColor(pixelDistance, ImageColorizer::REDTOBLUE);
			uint rgb = color.rgb();

//			img.data[2*index] = distanceMsb;
//			img.data[2*index + 1] = distanceLsb;
			img.data[3*index] = rgb >> 16;
			img.data[3*index + 1] = rgb >> 8;
			img.data[3*index + 2] = rgb;

			index++;
    }
  }

	distance_image_publisher.publish(img);
}
