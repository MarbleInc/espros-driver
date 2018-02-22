#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "data_connection.h"
#include "cmd_connection.h"
#include "image_colorizer.h"
#include "data_header.h"
#include <QSettings>
#include "settings.h"
#include "controller.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

  enum
  {
    STATE_NORMAL,
    STATE_STREAM_DIST_AMP,
    STATE_STREAM_DISTANCE,
    STATE_STREAM_GRAYSCALE,
    STATE_CALIBRATE
  };

public:
  explicit MainWindow(Controller &controller, Settings &settings, QWidget *parent = 0);
  ~MainWindow();

private Q_SLOTS:
  void receivedMeasurementData(const char *pData, DataHeader &dataHeader);
  void updateFps(const unsigned int fps);

  void connected();
  void disconnected();

  void onUserDataChanged();

  void onPushButtonDistAmpClicked();
  void onPushButtonStopStreamClicked();
  void onPushButtonDistanceClicked();
  void onPushButtonGrayscaleClicked();
  void onPushButtonCalibrationClicked();
  void onPushButtonCalibrateSystemOffsetClicked();

protected:
  void closeEvent (QCloseEvent *event  __attribute__((unused)));

private:
  void showDistanceAmplitude(const char *pData, DataHeader &dataHeader);
  void showDistance(const char *pData, DataHeader &dataHeader);
  void showGrayscale(const char *pData, DataHeader &dataHeader);
  void restoreSettings();
  void setState(const int state);

  Ui::MainWindow *ui;
  Controller &controller;
  Settings &settings;
};

#endif // MAINWINDOW_H
