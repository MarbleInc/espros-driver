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

	if (showDistance) {
		distance_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance/image_raw", 100);
		distance_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_distance/camera_info", 100);
	}

	if (showDistanceColor) {
		distance_color_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance_color/image_raw", 100);
		distance_color_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_distance_color/camera_info", 100);
	}

	if (showAmplitude) {
		amplitude_image_publisher = n.advertise<sensor_msgs::Image>("espros_amplitude/image_raw", 100);
		amplitude_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_amplitude/camera_info", 100);
	}

	if (showInterleave) {
		interleave_image_publisher = n.advertise<sensor_msgs::Image>("espros_interleave/image_raw", 100);
		interleave_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_interleave/camera_info", 100);
	}

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

	// read .launch params
	nh.param("espros_data", esprosData, -1);

	nh.param("show_distance", showDistance, 1);
	nh.param("show_distance_color", showDistanceColor, 0);
	nh.param("show_amplitude", showAmplitude, 0);
	nh.param("show_interleave", showInterleave, 0);

	nh.param("confidence_bits", confidenceBits, 1);

	//process console params
	if (-1 != esprosData) {
		showDistance = 0;
		showDistanceColor = 0;
		showAmplitude = 0;
		showInterleave = 0;

		switch(esprosData) {
			case 0: showDistance = 1;
				break;
			case 1: showDistanceColor = 1;
				break;
			case 2: showAmplitude = 1;
				break;
			case 3: showInterleave = 1;
				break;
		}
	}

	// determine data to request
	if ( ((showDistance || showDistanceColor) && showAmplitude) || showInterleave ) {
		fetchType = FETCH_INTERLEAVE;
	} else if (showAmplitude) {
		fetchType = FETCH_AMPLITUDE;
	} else {
		fetchType = FETCH_DISTANCE;
	}

	// set settings
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
	std::cout << "QNode: tcp connected" << std::endl;

  controller.sendAllSettingsToCamera();

	switch (fetchType) {
		case FETCH_DISTANCE:
			std::cout << "Requesting distance..." << std::endl;
			fetchType = FETCH_DISTANCE;
			controller.requestDistance(true); // stream
			break;
		case FETCH_AMPLITUDE:
			std::cout << "Requesting amplitude..." << std::endl;
			fetchType = FETCH_AMPLITUDE;
			controller.requestGrayscale(true); // stream
			break;
		case FETCH_INTERLEAVE:
			std::cout << "Requesting interleaved distance and amplitude..." << std::endl;
			fetchType = FETCH_INTERLEAVE;
			controller.requestDistanceAmplitude(true); // stream
			break;
	}

}

void QNode::tcpDisconnected() {
	std::cout << "QNode: tcp disconnected" << std::endl;
}

void QNode::renderData(const char *pData, DataHeader &dataHeader){
	if (showDistance) renderDistance(pData, dataHeader) ;
	if (showDistanceColor) renderDistanceColor(pData, dataHeader) ;
	if (showAmplitude) renderAmplitude(pData, dataHeader) ;
	if (showInterleave) renderInterleave(pData, dataHeader) ;
}

void QNode::setCameraInfo(const ros::Time time, sensor_msgs::CameraInfo *cInfo) {
	cInfo->header.stamp = time;
	cInfo->header.frame_id = FRAME_ID;
	cInfo->height = HEIGHT;
	cInfo->width = WIDTH;
	cInfo->distortion_model = DISTORTION_MODEL;
	cInfo->D = D;
	cInfo->K = K;
	cInfo->R = R;
	cInfo->P = P;
}

void QNode::renderDistance(const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::CameraInfo cInfo;
	sensor_msgs::Image img;

	ros::Time now = ros::Time::now();

	setCameraInfo(now, &cInfo);

	img.header.stamp = now;
	img.header.frame_id = FRAME_ID;

	img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 2;

	img.data.resize(img.step * img.height);

	for (int index = 0; index < (img.width * img.height); index++) {
		uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
		uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

		if (!confidenceBits) {
			distanceMsb = distanceMsb & 0b00111111;
		}

		img.data[2*index] = distanceMsb;
		img.data[2*index + 1] = distanceLsb;
	}

	distance_camera_info_publisher.publish(cInfo);
	distance_image_publisher.publish(img);
}

void QNode::renderDistanceColor(const char *pData, DataHeader &dataHeader)
{
	imageColorizerDistance.setRange(0, settings->getRange());

	sensor_msgs::CameraInfo cInfo;
	sensor_msgs::Image img;

	ros::Time now = ros::Time::now();

	setCameraInfo(now, &cInfo);

	img.header.stamp = now;
	img.header.frame_id = FRAME_ID;

	img.encoding = sensor_msgs::image_encodings::RGB8;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 3;

	img.data.resize(img.step * img.height);

	for (int index = 0; index < (img.width * img.height); index++) {
		uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
		uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

		distanceMsb = distanceMsb & 0b00111111; // scrub confidence bits

		unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;
		Color color = imageColorizerDistance.getColor(pixelDistance, ImageColorizer::RGB);

		img.data[3*index] = color.r;
		img.data[3*index + 1] = color.g;
		img.data[3*index + 2] = color.b;
	}

	distance_color_camera_info_publisher.publish(cInfo);
	distance_color_image_publisher.publish(img);
}

void QNode::renderAmplitude(const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::CameraInfo cInfo;
	sensor_msgs::Image img;

	ros::Time now = ros::Time::now();

	setCameraInfo(now, &cInfo);

	img.header.stamp = now;
	img.header.frame_id = FRAME_ID;

	img.encoding = sensor_msgs::image_encodings::MONO16;
	img.is_bigendian = 1; //true

	img.width = dataHeader.width;
	img.height = dataHeader.height;
	img.step = img.width * 2;

	img.data.resize(img.step * img.height);

	for (int index = 0; index < (img.width * img.height); index++) {

		uint8_t amplitudeMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
		uint8_t amplitudeLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

		if (!confidenceBits) {
			amplitudeMsb = amplitudeMsb & 0b00111111;
		}

		img.data[2*index] = amplitudeMsb;
		img.data[2*index + 1] = amplitudeLsb;
	}

	amplitude_camera_info_publisher.publish(cInfo);
	amplitude_image_publisher.publish(img);
}

void QNode::renderInterleave(const char *pData, DataHeader &dataHeader){
	sensor_msgs::CameraInfo cInfo;
	sensor_msgs::Image img;

	ros::Time now = ros::Time::now();

	setCameraInfo(now, &cInfo);

	img.header.stamp = now;
	img.header.frame_id = FRAME_ID;

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

		if (!confidenceBits) {
			amplitudeMsb = amplitudeMsb & 0b00001111;
			amplitudeMsb = amplitudeMsb & 0b00111111;
		}

		img.data[4*index] = amplitudeMsb;
		img.data[4*index + 1] = amplitudeLsb;
		img.data[4*index + 2] = distanceMsb;
		img.data[4*index + 3] = distanceLsb;
	}

	interleave_camera_info_publisher.publish(cInfo);
	interleave_image_publisher.publish(img);
}
