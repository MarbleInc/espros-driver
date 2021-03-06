#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/array.hpp>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include "controller.h"
#include "image_colorizer.h"
#include <diagnostic_msgs/KeyValue.h>


const std::string ESPROS32 = "ESPROS32";
const std::string FRAME_ID = "espros_base";

const int HEIGHT = 240;
const int WIDTH = 320;
const std::string DISTORTION_MODEL = "plumb_bob";
const std::vector<double> D = {-0.309386903830596, 0.06765995368321089, 0.005496266120711321, -0.001982504817185045, 0.0};
const boost::array<double, 9> K = {179.3276607258264, 0.0, 164.0886984119734, 0.0, 179.7358339944852, 108.31234252056599, 0.0, 0.0, 1.0};
const boost::array<double, 9> R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
const boost::array<double, 12> P = {122.7355728149414, 0.0, 161.61204232822092, 0.0, 0.0, 148.3640594482422, 105.1743229432941, 0.0, 0.0, 0.0, 1.0, 0.0};

const std::string INT_TIME_0 = "integration_time_0";
const std::string INT_TIME_1 = "integration_time_1";
const std::string INT_TIME_2 = "integration_time_2";
const std::string INT_TIME_GRAY = "integration_time_grayscale";
const std::string OFFSET = "offset";
const std::string MIN_AMP = "min_amplitude";
const std::string RANGE = "range";
const std::string CONFIDENCE = "confidence_bits";
const std::string ORIENT_VERTICAL = "orient_vertical";
const std::string ORIENT_HORIZONTAL = "orient_horizontal";

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

    enum {
      FETCH_DISTANCE,
      FETCH_GRAYSCALE,
      FETCH_INTERLEAVE
    };

public:
	QNode(int argc, char** argv, Controller &controller);
	virtual ~QNode();
	bool init();
	void run();
  void setSettings(const Settings *settings);
  void paramCallback(const diagnostic_msgs::KeyValueConstPtr& msg);


  private slots:
    void tcpConnected();
    void tcpDisconnected();
    void renderData(const char *pData, DataHeader &dataHeader);


signals:
    void rosShutdown();

private:
  void fetchParams();
  void setCameraInfo(const ros::Time time, sensor_msgs::CameraInfo *cInfo);
  void setImage(const ros::Time time, const int pixelBytes, sensor_msgs::Image *img);
  int getIndex(const int x, const int y, const int pixelBytes);

  void renderDistance(const ros::Time* now, const char *pData, DataHeader &dataHeader);
  void renderDistanceColor(const ros::Time* now, const char *pData, DataHeader &dataHeader);
  void renderGrayscale(const ros::Time* now, const char *pData, DataHeader &dataHeader);
  void renderInterleave(const ros::Time* now, const char *pData, DataHeader &dataHeader);
  void renderAmplitude(const ros::Time* now, const char *pData, DataHeader &dataHeader);

	int init_argc;
	char** init_argv;
  Controller &controller;
  Settings *settings;
  ImageColorizer imageColorizerDistance;

  ros::Subscriber param_subscriber;

  ros::Publisher distance_image_publisher;
  ros::Publisher distance_camera_info_publisher;

  ros::Publisher distance_color_image_publisher;
  ros::Publisher distance_color_camera_info_publisher;

  ros::Publisher grayscale_image_publisher;
  ros::Publisher grayscale_camera_info_publisher;

  ros::Publisher interleave_image_publisher;
  ros::Publisher interleave_camera_info_publisher;

  ros::Publisher amplitude_image_publisher;
  ros::Publisher amplitude_camera_info_publisher;

  int esprosData; // indicates single topic via console (rosrun)
  int confidenceBits; // include confidence bits in output
  int showDistance;
  int showDistanceColor;
  int showGrayscale;
  int showInterleave;
  int showAmplitude;
  int fetchType;
  int orientVertical;
  int orientHorizontal;
};

#endif
