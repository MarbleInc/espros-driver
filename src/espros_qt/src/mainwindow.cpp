#include "espros_qt/mainwindow.h"
#include "ui_mainwindow.h"
#include "espros_qt/image_colorizer.h"
#include <QImage>
#include <QApplication>
#include <QMessageBox>
#include "espros_qt/udp_data_connection.h"
#include "espros_qt/tcp_cmd_connection.h"
#include <QTimer>
#include "espros_qt/data_header.h"
#include "math.h"

//int argc, char** argv,
//    qnode(argc, argv)
MainWindow::MainWindow(Controller &controller, Settings &settings, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    controller(controller),
    settings(settings)

{
  ui->setupUi(this);

  //Restore the user settings
  restoreSettings();

  //Some stylesheet setups
  ui->pushButtonDistAmp->setStyleSheet("background-color: green");
  ui->pushButtonDistance->setStyleSheet("background-color: blue");
  ui->pushButtonStopStream->setStyleSheet("background-color: red");
  ui->labelFps->setStyleSheet("QLabel { color : red; }");

  //Signals from the controller to the main window
  connect(&controller, &Controller::receivedMeasurementData, this, &MainWindow::receivedMeasurementData);
  connect(&controller, &Controller::updateFps, this, &MainWindow::updateFps);
  connect(&controller, &Controller::connected, this, &MainWindow::connected);
  connect(&controller, &Controller::disconnected, this, &MainWindow::disconnected);

  //Signals from the GUI elements
  connect(ui->spinBoxIntTime0, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), &settings, &Settings::setIntegrationTime0);
  connect(ui->spinBoxIntTime1, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), &settings, &Settings::setIntegrationTime1);
  connect(ui->spinBoxIntTimeGrayscale, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), &settings, &Settings::setIntegrationTimeGrayscale);
  connect(ui->spinBoxOffset, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), &settings, &Settings::setOffset);
  connect(ui->spinBoxMinAmplitude, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), &settings, &Settings::setMinAmplitude);
  connect(ui->spinBoxRange, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), &settings, &Settings::setRange);
  connect(ui->textEditUserData, static_cast<void (QTextEdit::*)()>(&QTextEdit::textChanged), this, &MainWindow::onUserDataChanged);

  //Buttons
  connect(ui->pushButtonDistAmp, static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked), this, &MainWindow::onPushButtonDistAmpClicked);
  connect(ui->pushButtonDistance, static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked), this, &MainWindow::onPushButtonDistanceClicked);
  connect(ui->pushButtonStopStream, static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked), this, &MainWindow::onPushButtonStopStreamClicked);
  connect(ui->pushButtonCalibration, static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked), this, &MainWindow::onPushButtonCalibrationClicked);
  connect(ui->pushButtonSystemOffset, static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked), this, &MainWindow::onPushButtonCalibrateSystemOffsetClicked);
  connect(ui->pushButtonGrayscale, static_cast<void (QPushButton::*)(bool)>(&QPushButton::clicked), this, &MainWindow::onPushButtonGrayscaleClicked);

  setState(STATE_NORMAL);

  disconnected();

  ui->dockWidgetContents->setSettings(&settings);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::closeEvent (QCloseEvent *event  __attribute__((unused)))
{
  controller.close();
}

void MainWindow::restoreSettings()
{
  ui->spinBoxIntTime0->setValue(settings.getIntegrationTime0());
  ui->spinBoxIntTime1->setValue(settings.getIntegrationTime1());
  ui->spinBoxIntTimeGrayscale->setValue(settings.getIntegrationTimeGrayscale());
  ui->spinBoxOffset->setValue(settings.getOffset());
  ui->spinBoxMinAmplitude->setValue(settings.getMinAmplitude());
  ui->spinBoxRange->setValue(settings.getRange());

  QString string(settings.getUserData());
  ui->textEditUserData->setText(string);
}

void MainWindow::setState(const int state)
{
  switch(state)
  {
    case STATE_NORMAL:
      ui->checkBoxStream->setEnabled(true);
      ui->pushButtonStopStream->setEnabled(false);
      ui->pushButtonDistance->setChecked(false);
      ui->pushButtonDistAmp->setChecked(false);
      ui->pushButtonGrayscale->setChecked(false);
      ui->pushButtonCalibration->setEnabled(true);
      ui->pushButtonSystemOffset->setEnabled(true);
      break;
    case STATE_STREAM_DISTANCE:
      ui->checkBoxStream->setEnabled(false);
      ui->pushButtonStopStream->setEnabled(true);
      ui->pushButtonDistance->setChecked(true);
      ui->pushButtonDistAmp->setChecked(false);
      ui->pushButtonGrayscale->setChecked(false);
      ui->pushButtonCalibration->setEnabled(false);
      ui->pushButtonSystemOffset->setEnabled(false);
      break;
    case STATE_STREAM_DIST_AMP:
      ui->checkBoxStream->setEnabled(false);
      ui->pushButtonStopStream->setEnabled(true);
      ui->pushButtonDistance->setChecked(false);
      ui->pushButtonDistAmp->setChecked(true);
      ui->pushButtonGrayscale->setChecked(false);
      ui->pushButtonCalibration->setEnabled(false);
      ui->pushButtonSystemOffset->setEnabled(false);
      break;
    case STATE_STREAM_GRAYSCALE:
      ui->checkBoxStream->setEnabled(false);
      ui->pushButtonStopStream->setEnabled(true);
      ui->pushButtonDistance->setChecked(false);
      ui->pushButtonDistAmp->setChecked(false);
      ui->pushButtonGrayscale->setChecked(true);
      ui->pushButtonCalibration->setEnabled(false);
      ui->pushButtonSystemOffset->setEnabled(false);
      break;
  }
}

void MainWindow::receivedMeasurementData(const char *pData, DataHeader &dataHeader)
{
  ui->widgetHeader->showHeaderData(dataHeader);

  ui->dockWidgetContents->showData(pData, dataHeader);
}

void MainWindow::updateFps(const unsigned int fps)
{
  if (fps == 0)
  {
    ui->labelFps->setStyleSheet("QLabel { color : red; }");
  }
  else
  {
    ui->labelFps->setStyleSheet("QLabel { color : green; }");
  }
  ui->labelFps->setText(QString::number(fps) + "Fps");
}

void MainWindow::connected()
{
  ui->labelStatus->setStyleSheet("QLabel { color : green; }");
  ui->labelStatus->setText("Connected");


  //Update the camera on connection
  controller.sendAllSettingsToCamera();
}

void MainWindow::disconnected()
{
  ui->labelStatus->setStyleSheet("QLabel { color : red; }");
  ui->labelStatus->setText("Disconnected");
}

void MainWindow::onUserDataChanged()
{
  QByteArray userData = ui->textEditUserData->toPlainText().toLocal8Bit();
  settings.setUserData(userData);
}

void MainWindow::onPushButtonDistanceClicked()
{
  bool isStream = ui->checkBoxStream->isChecked();

  if (isStream)
  {
    setState(STATE_STREAM_DISTANCE);
  }
  else
  {
    setState(STATE_NORMAL);
  }

  controller.requestDistance(isStream);
}

void MainWindow::onPushButtonGrayscaleClicked()
{
  bool isStream = ui->checkBoxStream->isChecked();

  if (isStream)
  {
    setState(STATE_STREAM_GRAYSCALE);
  }
  else
  {
    setState(STATE_NORMAL);
  }

  controller.requestGrayscale(isStream);
}

void MainWindow::onPushButtonDistAmpClicked()
{
  bool isStream = ui->checkBoxStream->isChecked();

  if (isStream)
  {
    setState(STATE_STREAM_DIST_AMP);
  }
  else
  {
    setState(STATE_NORMAL);
  }

  controller.requestDistanceAmplitude(isStream);
}

void MainWindow::onPushButtonCalibrationClicked()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, "Start Calibration", "Start Calibration? Current calibration will be lost.",QMessageBox::Yes|QMessageBox::No);
  if (reply == QMessageBox::Yes)
  {
    setState(STATE_CALIBRATE);
    controller.calibrate();
  }
}

void MainWindow::onPushButtonCalibrateSystemOffsetClicked()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(this, "Calibrate System Offset", "Place the camera at 60cm from a wall",QMessageBox::Ok|QMessageBox::Cancel);
  if (reply == QMessageBox::Ok)
  {
    setState(STATE_CALIBRATE);
    controller.calibrateSystemOffset();
  }
}

void MainWindow::onPushButtonStopStreamClicked()
{
  setState(STATE_NORMAL);
  controller.stopStream();
}
