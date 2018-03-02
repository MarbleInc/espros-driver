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
#include <sensor_msgs/CameraInfo.h>
#include "espros_nogui/qnode.hpp"

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
		connect(&controller, &Controller::receivedMeasurementData, this, &QNode::renderData);
	}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"espros_nogui");
	if ( ! ros::master::check() ) {
		return false;
	}
	fetchParams();
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	distance_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance", 100);
	amplitude_image_publisher = n.advertise<sensor_msgs::Image>("espros_amplitude", 100);
	amplitude_distance_image_publisher = n.advertise<sensor_msgs::Image>("espros_amplitude_distance", 100);
	camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_camera_info", 100);
	start();
	return true;
}

//this is really just a shut down handler
void QNode::run() {
	ros::Rate loop_rate(1);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown..." << std::endl;
	emit rosShutdown();
}

void QNode::fetchParams() {
	ros::NodeHandle nh("~");
	nh.param("espros_data", esprosData, 0);

	int intTime0, intTime1, intTime2, intTimeGray, offset, minAmp, range;

	if (nh.getParam("integration_time_0", intTime0)) {
		settings->setIntegrationTime0(intTime0);
	}

	if (nh.getParam("integration_time_1", intTime1)) {
		settings->setIntegrationTime1(intTime1);
	}

	if (nh.getParam("integration_time_2", intTime2)) {
		settings->setIntegrationTime2(intTime2);
	}

	if (nh.getParam("integration_time_grayscale", intTimeGray)) {
		settings->setIntegrationTimeGrayscale(intTimeGray);
	}

	if (nh.getParam("offset", offset)) {
		settings->setOffset(offset);
	}

	if (nh.getParam("min_amplitude", minAmp)) {
		settings->setMinAmplitude(minAmp);
	}

	if (nh.getParam("range", range)) {
		settings->setRange(range);
	}

}

void QNode::setSettings(const Settings *settings)
{
  this->settings = (Settings *)settings;
}

void QNode::tcpConnected() {
	std_msgs::String msg;
	msg.data = "QNode: tcp connected";
	chatter_publisher.publish(msg);

	std::cout << "QNode: tcp connected" << std::endl;

  controller.sendAllSettingsToCamera();

	if (0 == esprosData) {
		std::cout << "Requesting distance..." << std::endl;
		controller.requestDistance(true); // stream
	} else if (1 == esprosData) {
		std::cout << "Requesting amplitude..." << std::endl;
		controller.requestGrayscale(true); // stream
	} else {
		std::cout << "Requesting distance and amplitude..." << std::endl;
		controller.requestDistanceAmplitude(true); // stream
	}
}

void QNode::tcpDisconnected() {
	std_msgs::String msg;
	msg.data = "QNode: tcp disconnected";
	chatter_publisher.publish(msg);

	std::cout << "QNode: tcp disconnected" << std::endl;
}

void QNode::renderData(const char *pData, DataHeader &dataHeader){
	if (0 == esprosData) {
		showDistance(pData, dataHeader);
	} else if (1 == esprosData) {
		showGrayscale(pData, dataHeader);
	} else {
		showBoth(pData, dataHeader);
	}
}

void QNode::showDistance(const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::CameraInfo cInfo;
	sensor_msgs::Image img;

	ros::Time now = ros::Time::now();

	cInfo.header.stamp = now;
	cInfo.header.frame_id = FRAME_ID;

	img.header.stamp = now;
	img.header.frame_id = FRAME_ID;

	img.encoding = sensor_msgs::image_encodings::MONO16;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 2;

	img.data.resize(img.step * img.height);

	for (int index = 0; index < (img.width * img.height); index++) {
		uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
		uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

		img.data[2*index] = distanceMsb;
		img.data[2*index + 1] = distanceLsb;
	}

	camera_info_publisher.publish(cInfo);
	distance_image_publisher.publish(img);
}

void QNode::showGrayscale(const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::Image img;

	img.header.stamp = ros::Time::now();
	img.header.frame_id = "1";

	img.encoding = sensor_msgs::image_encodings::MONO16;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 2;

	img.data.resize(img.step * img.height);

	for (int index = 0; index < (img.width * img.height); index++) {

		uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
		uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

		img.data[2*index] = distanceMsb;
		img.data[2*index + 1] = distanceLsb;
	}

	amplitude_image_publisher.publish(img);
}

void QNode::showBoth(const char *pData, DataHeader &dataHeader){
	sensor_msgs::Image img;

	img.header.stamp = ros::Time::now();
	img.header.frame_id = "1";

	img.encoding = ESPROS32;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 4;

	img.data.resize(img.step * img.height);

	for (int index = 0; index < (img.width * img.height); index++) {
		uint8_t amplitudeMsb = (uint8_t) pData[4*index+3+dataHeader.offset];
		uint8_t amplitudeLsb = (uint8_t) pData[4*index+2+dataHeader.offset];
		uint8_t distanceMsb = (uint8_t) pData[4*index+1+dataHeader.offset];
		uint8_t distanceLsb = (uint8_t) pData[4*index+0+dataHeader.offset];

		img.data[4*index] = amplitudeMsb;
		img.data[4*index + 1] = amplitudeLsb;
		img.data[4*index + 2] = distanceMsb;
		img.data[4*index + 3] = distanceLsb;
	}

	amplitude_distance_image_publisher.publish(img);
}
