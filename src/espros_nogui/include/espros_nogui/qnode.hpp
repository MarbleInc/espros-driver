#ifndef QNODE_H
#define QNODE_H

/**
 * @file /includeespros_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "controller.h"
#include "image_colorizer.h"


const std::string ESPROS32 = "ESPROS32";
const std::string FRAME_ID = "espros_base";

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv, Controller &controller);
	virtual ~QNode();
	bool init();
	void run();
  void setSettings(const Settings *settings);


  private slots:
    void tcpConnected();
    void tcpDisconnected();
    void renderData(const char *pData, DataHeader &dataHeader);


signals:
    void rosShutdown();

private:
  void fetchParams();
  void renderDistance(const char *pData, DataHeader &dataHeader);
  void renderDistanceColor(const char *pData, DataHeader &dataHeader);
  void renderAmplitude(const char *pData, DataHeader &dataHeader);
  void renderInterleave(const char *pData, DataHeader &dataHeader);

	int init_argc;
	char** init_argv;
  Controller &controller;
  Settings *settings;
  ImageColorizer imageColorizerDistance;

  ros::Publisher distance_image_publisher;
  ros::Publisher distance_camera_info_publisher;

  ros::Publisher distance_color_image_publisher;
  ros::Publisher distance_color_camera_info_publisher;

  ros::Publisher amplitude_image_publisher;
  ros::Publisher amplitude_camera_info_publisher;

  ros::Publisher interleave_image_publisher;
  ros::Publisher interleave_camera_info_publisher;

  int esprosData; // indicates single topic via console (rosrun)
  int confidenceBits; // include confidence bits in output
  int showDistance;
  int showDistanceColor;
  int showAmplitude;
  int showInterleave;
};

#endif
