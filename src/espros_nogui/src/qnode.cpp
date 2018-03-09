#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <stdlib.h>
#include "espros_nogui/qnode.hpp"

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

	param_subscriber = n.subscribe<diagnostic_msgs::KeyValue>("espros_param", 100, &QNode::paramCallback, this);

	if (showGrayscale) {
		grayscale_image_publisher = n.advertise<sensor_msgs::Image>("espros_grayscale/image_raw", 100);
		grayscale_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_grayscale/camera_info", 100);
	} else {
		if (showDistance) {
			distance_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance/image_raw", 100);
			distance_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_distance/camera_info", 100);
		}

		if (showDistanceColor) {
			distance_color_image_publisher = n.advertise<sensor_msgs::Image>("espros_distance_color/image_raw", 100);
			distance_color_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_distance_color/camera_info", 100);
		}

		if (showInterleave) {
			interleave_image_publisher = n.advertise<sensor_msgs::Image>("espros_interleave/image_raw", 100);
			interleave_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_interleave/camera_info", 100);
		}

		if (showAmplitude) {
			amplitude_image_publisher = n.advertise<sensor_msgs::Image>("espros_amplitude/image_raw", 100);
			amplitude_camera_info_publisher = n.advertise<sensor_msgs::CameraInfo>("espros_amplitude/camera_info", 100);
		}
	}

	start();
	return true;
}

//this is really just a shut down handler
void QNode::run() {
	ros::Rate loop_rate(30);
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
	nh.param("show_grayscale", showGrayscale, 0);
	nh.param("show_interleave", showInterleave, 0);
	nh.param("show_amplitude", showAmplitude, 0);

	nh.param("confidence_bits", confidenceBits, 1);

	nh.param("orient_vertical", orientVertical, 0);
	nh.param("orient_horizontal", orientHorizontal, 0);

	//process console params
	if (-1 != esprosData) {
		showDistance = 0;
		showDistanceColor = 0;
		showGrayscale = 0;
		showInterleave = 0;
		showAmplitude = 0;

		switch(esprosData) {
			case 0: showDistance = 1;
				break;
			case 1: showDistanceColor = 1;
				break;
			case 2: showGrayscale = 1;
				break;
			case 3: showInterleave = 1;
				break;
			case 4: showAmplitude = 1;
				break;
		}
	}

	if (showGrayscale) {
		fetchType = FETCH_GRAYSCALE;
	} else if (showInterleave || showAmplitude) {
		fetchType = FETCH_INTERLEAVE;
	} else {
		fetchType = FETCH_DISTANCE;
	}

	// set settings
	int intTime0, intTime1, intTime2, intTimeGray, offset, minAmp, range;

	if (nh.getParam(INT_TIME_0, intTime0)) {
		settings->setIntegrationTime0(intTime0);
	}

	if (nh.getParam(INT_TIME_1, intTime1)) {
		settings->setIntegrationTime1(intTime1);
	}

	if (nh.getParam(INT_TIME_2, intTime2)) {
		settings->setIntegrationTime2(intTime2);
	}

	if (nh.getParam(INT_TIME_GRAY, intTimeGray)) {
		settings->setIntegrationTimeGrayscale(intTimeGray);
	}

	if (nh.getParam(OFFSET, offset)) {
		settings->setOffset(offset);
	}

	if (nh.getParam(MIN_AMP, minAmp)) {
		settings->setMinAmplitude(minAmp);
	}

	if (nh.getParam(RANGE, range)) {
		settings->setRange(range);
	}

}

void QNode::paramCallback(const diagnostic_msgs::KeyValueConstPtr& msg) {
	std::string key = msg->key;
	std::string value = msg->value;

	int num = atoi(value.c_str());
	if (0 == num && "0" != value) {
		num = -1;
	}

	bool valid = false;
	if(0 <= num) {
		valid = true;
		if (INT_TIME_0 == key) {
			settings->setIntegrationTime0(num);
		} else if (INT_TIME_1 == key) {
			settings->setIntegrationTime1(num);
		} else if (INT_TIME_2 == key) {
			settings->setIntegrationTime2(num);
		} else if (INT_TIME_GRAY == key) {
			settings->setIntegrationTimeGrayscale(num);
		} else if (OFFSET == key) {
			settings->setOffset(num);
		} else if (MIN_AMP == key) {
			settings->setMinAmplitude(num);
		} else if (RANGE == key) {
			settings->setRange(num);
		} else if (CONFIDENCE == key) {
			confidenceBits = num;
		} else if (ORIENT_VERTICAL == key) {
			orientVertical = num;
		} else if (ORIENT_HORIZONTAL == key) {
			orientHorizontal = num;
		} else {
			valid = false;
		}
	}

	if (valid) {
		controller.sendAllSettingsToCamera();
		std::cout << "Set parameter: " << key << "=" << value << std::endl;
	} else {
		std::cout << "Invalid parameter: " << key << "=" << value << std::endl;
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
		case FETCH_GRAYSCALE:
			std::cout << "Requesting grayscale..." << std::endl;
			fetchType = FETCH_GRAYSCALE;
			controller.requestGrayscale(true); // stream
			break;
		case FETCH_INTERLEAVE:
			std::cout << "Requesting interleaved distance and amplitude..." << std::endl;
			fetchType = FETCH_INTERLEAVE;
			controller.requestDistanceAmplitude(true); // stream
			break;
		case FETCH_AMPLITUDE:
			std::cout << "Requesting amplitude..." << std::endl;
			fetchType = FETCH_AMPLITUDE;
			controller.requestAmplitude(true); // stream
			break;
	}

}

void QNode::tcpDisconnected() {
	std::cout << "QNode: tcp disconnected" << std::endl;
}

void QNode::renderData(const char *pData, DataHeader &dataHeader){
	const ros::Time now = ros::Time::now();

	if (showGrayscale) {
		renderGrayscale(&now, pData, dataHeader);
	} else {
		if (showDistance) renderDistance(&now, pData, dataHeader);
		if (showDistanceColor) renderDistanceColor(&now, pData, dataHeader);
		if (showInterleave) renderInterleave(&now, pData, dataHeader);
		if (showAmplitude) renderAmplitude(&now, pData, dataHeader);
	}
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

void QNode::setImage(const ros::Time time, const int pixelBytes, sensor_msgs::Image *img) {
	img->header.stamp = time;
	img->header.frame_id = FRAME_ID;
	img->is_bigendian = 1; //true
	img->width = WIDTH;
	img->height = HEIGHT;
	img->step = img->width * pixelBytes;
	img->data.resize(img->step * img->height);
}

int QNode::getIndex(const int x, const int y, const int pixelBytes) {
	int ox, oy; // oriented values

	// flip vertical
	if (orientVertical) {
		oy = HEIGHT - 1 - y;
	} else {
		oy = y;
	}

	// flip horizontal
	if (orientHorizontal) {
		ox = WIDTH - 1 - x;
	} else {
		ox = x;
	}

	// compute array index
	int index = ((oy * WIDTH) + ox) * pixelBytes;

	return index;
}

void QNode::renderDistance(const ros::Time* now, const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::Image img;
	sensor_msgs::CameraInfo cInfo;

	int pixelBytes = 2;

	setCameraInfo(*now, &cInfo);
	setImage(*now, pixelBytes, &img);

	img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  uint16_t* data_16_arr = reinterpret_cast<uint16_t*>(&img.data[0]);

	uint8_t distanceMsb;
	uint8_t distanceLsb;
	int readIndex = 0;
	int writeIndex;

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			// read data
			if (FETCH_INTERLEAVE == fetchType) {
				distanceMsb = (uint8_t) pData[4*readIndex+1+dataHeader.offset];
				distanceLsb = (uint8_t) pData[4*readIndex+0+dataHeader.offset];
			} else {
				distanceMsb = (uint8_t) pData[2*readIndex+1+dataHeader.offset];
				distanceLsb = (uint8_t) pData[2*readIndex+0+dataHeader.offset];
			}

			unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;

			// remove confidence bits
			if (!confidenceBits) {
				if (16000 < pixelDistance) {
					distanceMsb = 0;
					distanceLsb = 0;
					pixelDistance = 0;
				}

				distanceMsb = distanceMsb & 0b00111111;
			}

			// flip image
			writeIndex = getIndex(x, y, 1);

      data_16_arr[writeIndex] = (uint16_t) pixelDistance;

			readIndex++;
		}
	}

	distance_camera_info_publisher.publish(cInfo);
	distance_image_publisher.publish(img);
}

void QNode::renderDistanceColor(const ros::Time* now, const char *pData, DataHeader &dataHeader)
{
	imageColorizerDistance.setRange(0, settings->getRange());

	sensor_msgs::Image img;
	sensor_msgs::CameraInfo cInfo;

	int pixelBytes = 3;

	setCameraInfo(*now, &cInfo);
	setImage(*now, pixelBytes, &img);

	img.encoding = sensor_msgs::image_encodings::RGB8;

	uint8_t distanceMsb;
	uint8_t distanceLsb;
	int readIndex = 0;
	int writeIndex;
	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			if (FETCH_INTERLEAVE == fetchType) {
				distanceMsb = (uint8_t) pData[4*readIndex+1+dataHeader.offset];
				distanceLsb = (uint8_t) pData[4*readIndex+0+dataHeader.offset];
			} else {
				distanceMsb = (uint8_t) pData[2*readIndex+1+dataHeader.offset];
				distanceLsb = (uint8_t) pData[2*readIndex+0+dataHeader.offset];
			}

			unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;
			Color color = imageColorizerDistance.getColor(pixelDistance, ImageColorizer::RGB);

			// remove confidence bits
			if (!confidenceBits) {
				if (16000 < pixelDistance) {
					distanceMsb = 0;
					distanceLsb = 0;
					pixelDistance = 0;
				}

				distanceMsb = distanceMsb & 0b00111111;
			}

			// flip image
			writeIndex = getIndex(x, y, pixelBytes);

			img.data[writeIndex] = color.r;
			img.data[writeIndex + 1] = color.g;
			img.data[writeIndex + 2] = color.b;

			readIndex++;
		}
	}

	distance_color_camera_info_publisher.publish(cInfo);
	distance_color_image_publisher.publish(img);
}

void QNode::renderGrayscale(const ros::Time* now, const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::Image img;
	sensor_msgs::CameraInfo cInfo;

	int pixelBytes = 2;

	setCameraInfo(*now, &cInfo);
	setImage(*now, pixelBytes, &img);

	img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	uint16_t* data_16_arr = reinterpret_cast<uint16_t*>(&img.data[0]);


	uint8_t grayscaleMsb;
	uint8_t grayscaleLsb;
	int readIndex = 0;
	int writeIndex;

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			grayscaleMsb = (uint8_t) pData[2*readIndex+1+dataHeader.offset];
			grayscaleLsb = (uint8_t) pData[2*readIndex+0+dataHeader.offset];

			unsigned int pixelShade = (grayscaleMsb << 8) + grayscaleLsb;

			// remove confidence bits
			if (!confidenceBits) {
				if (16000 < pixelShade) {
					grayscaleMsb = 0;
					grayscaleLsb = 0;
					pixelShade = 0;
				}

				grayscaleMsb = grayscaleMsb & 0b00111111;
			}

			// flip image
			writeIndex = getIndex(x, y, 1);

			data_16_arr[writeIndex] = (uint16_t) pixelShade;

			readIndex++;
		}
	}

	grayscale_camera_info_publisher.publish(cInfo);
	grayscale_image_publisher.publish(img);
}

void QNode::renderInterleave(const ros::Time* now, const char *pData, DataHeader &dataHeader){
	sensor_msgs::Image img;
	sensor_msgs::CameraInfo cInfo;

	int pixelBytes = 4;

	setCameraInfo(*now, &cInfo);
	setImage(*now, pixelBytes, &img);

	img.encoding = ESPROS32;
	uint16_t* data_16_arr = reinterpret_cast<uint16_t*>(&img.data[0]);

	int readIndex = 0;
	int writeIndex;

	uint8_t amplitudeMsb, amplitudeLsb, distanceMsb, distanceLsb;
	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			amplitudeMsb = (uint8_t) pData[4*readIndex+3+dataHeader.offset];
			amplitudeLsb = (uint8_t) pData[4*readIndex+2+dataHeader.offset];
			distanceMsb = (uint8_t) pData[4*readIndex+1+dataHeader.offset];
			distanceLsb = (uint8_t) pData[4*readIndex+0+dataHeader.offset];

			unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;
			unsigned int pixelAmplitude = (amplitudeMsb << 8) + amplitudeLsb;

			if (!confidenceBits) {
				amplitudeMsb = amplitudeMsb & 0b00001111;
				distanceMsb = distanceMsb & 0b00111111;
			}

			// remove confidence bits
			if (!confidenceBits) {
				if (16000 < pixelDistance) {
					distanceMsb = 0;
					distanceLsb = 0;
					amplitudeMsb = 0;
					amplitudeLsb = 0;

					pixelDistance = 0;
					pixelAmplitude = 0;
				}

				amplitudeMsb = amplitudeMsb & 0b00001111;
				distanceMsb = distanceMsb & 0b00111111;
			}

			// flip image
			writeIndex = getIndex(x, y, 1);

			data_16_arr[writeIndex] = (uint16_t) pixelAmplitude;
			data_16_arr[writeIndex + 1] = (uint16_t) pixelDistance;
			
			readIndex++;
		}
	}

	interleave_camera_info_publisher.publish(cInfo);
	interleave_image_publisher.publish(img);
}

void QNode::renderAmplitude(const ros::Time* now, const char *pData, DataHeader &dataHeader)
{
	sensor_msgs::Image img;
	sensor_msgs::CameraInfo cInfo;

	int pixelBytes = 2;

	setCameraInfo(*now, &cInfo);
	setImage(*now, pixelBytes, &img);

	img.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  uint16_t* data_16_arr = reinterpret_cast<uint16_t*>(&img.data[0]);

	uint8_t amplitudeMsb;
	uint8_t amplitudeLsb;
	int readIndex = 0;
	int writeIndex;

	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			// read data
			if (FETCH_INTERLEAVE == fetchType) {
				amplitudeMsb = (uint8_t) pData[4*readIndex+1+dataHeader.offset];
				amplitudeLsb = (uint8_t) pData[4*readIndex+0+dataHeader.offset];
			} else {
				amplitudeMsb = (uint8_t) pData[2*readIndex+1+dataHeader.offset];
				amplitudeLsb = (uint8_t) pData[2*readIndex+0+dataHeader.offset];
			}

			unsigned int pixelAmplitude = (amplitudeMsb << 8) + amplitudeLsb;

			// remove confidence bits
			if (!confidenceBits) {
				if (16000 < pixelAmplitude) {
					amplitudeMsb = 0;
					amplitudeLsb = 0;
					pixelAmplitude = 0;
				}

				amplitudeMsb = amplitudeMsb & 0b00111111;
			}

			// flip image
			writeIndex = getIndex(x, y, 1);

      data_16_arr[writeIndex] = (uint16_t) pixelAmplitude;

			readIndex++;
		}
	}

	amplitude_camera_info_publisher.publish(cInfo);
	amplitude_image_publisher.publish(img);
}
