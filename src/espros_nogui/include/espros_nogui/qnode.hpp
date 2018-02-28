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

#include <bitset>

const std::string ESPROS32 = "ESPROS32";

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
  void showDistance(const char *pData, DataHeader &dataHeader);
  void showGrayscale(const char *pData, DataHeader &dataHeader);
  void showBoth(const char *pData, DataHeader &dataHeader);

	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Publisher distance_image_publisher;
  ros::Publisher amplitude_image_publisher;
  ros::Publisher amplitude_distance_image_publisher;
  Controller &controller;
  Settings *settings;
  int esprosData;
};

#endif
