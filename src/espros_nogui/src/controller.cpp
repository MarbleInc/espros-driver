#include "espros_nogui/controller.h"


Controller::Controller(Settings &settings, Interface &interface):settings(settings), interface(interface)
{
  //Signals from the settings
  connect(&settings, static_cast<void (Settings::*)()>(&Settings::offsetChanged), this, &Controller::onOffsetChanged);
  connect(&settings, static_cast<void (Settings::*)()>(&Settings::integrationTimeChanged), this, &Controller::onIntegrationTimesChanged);
  connect(&settings, static_cast<void (Settings::*)()>(&Settings::minAmplitudeChanged), this, &Controller::onMinAmplitudeChanged);

  //Signals from the interface
  connect(&interface, &Interface::receivedMeasurementData, this, &Controller::receivedData);
  connect(&interface, &Interface::updateFps, this, &Controller::onFpsUpdate);
  connect(&interface, &Interface::connected, this, &Controller::connected);
  connect(&interface, &Interface::disconnected, this, &Controller::disconnected);
}

//Integration time changed, update the camera
void Controller::onIntegrationTimesChanged()
{
  uint16_t integrationTimes3d[3];
  uint16_t integrationTimeGrayscale;

  integrationTimes3d[0] = settings.getIntegrationTime0();
  integrationTimes3d[1] = settings.getIntegrationTime1();
  integrationTimes3d[2] = settings.getIntegrationTime2();
  integrationTimeGrayscale = settings.getIntegrationTimeGrayscale();

  QByteArray userData = settings.getUserData();
  interface.setIntegrationTimes(integrationTimes3d, integrationTimeGrayscale, userData);
}

//Offset changed, update the camera
void Controller::onOffsetChanged()
{
  QByteArray userData = settings.getUserData();
  int16_t offset = settings.getOffset();
  interface.setOffset(offset, userData);
}

//Min. amplitude changed, update the camera
void Controller::onMinAmplitudeChanged()
{
  QByteArray userData = settings.getUserData();
  uint16_t minAmplitude = settings.getMinAmplitude();
  interface.setMinAmplitude(minAmplitude, userData);
}

//Send all settings to the camera, for example when the connection is established
void Controller::sendAllSettingsToCamera()
{
  uint16_t integrationTimes3d[3];
  uint16_t integrationTimeGrayscale;

  integrationTimes3d[0] = settings.getIntegrationTime0();
  integrationTimes3d[1] = settings.getIntegrationTime1();
  integrationTimes3d[2] = settings.getIntegrationTime2();
  integrationTimeGrayscale = settings.getIntegrationTimeGrayscale();
  interface.setIntegrationTimes(integrationTimes3d, integrationTimeGrayscale);

  int16_t offset = settings.getOffset();
  interface.setOffset(offset);

  uint16_t minAmplitude = settings.getMinAmplitude();
  interface.setMinAmplitude(minAmplitude);
}

//Wrapper function to request the distance from the interface
void Controller::requestDistance(const bool isStream)
{
  QByteArray userData = settings.getUserData();
  interface.requestDistance(isStream, userData);
}

//Wrapper function to request grayscale from the interface
void Controller::requestGrayscale(const bool isStream)
{
  QByteArray userData = settings.getUserData();
  interface.requestGrayscale(isStream, userData);
}

//Wrapper function to request distance and amplitude from the interface
void Controller::requestDistanceAmplitude(const bool isStream)
{
  QByteArray userData = settings.getUserData();
  interface.requestDistanceAmplitude(isStream, userData);
}

//Wrapper function to stop a stream
void Controller::stopStream()
{
  QByteArray userData = settings.getUserData();
  interface.stopStream(userData);
}

void Controller::calibrate()
{
  QByteArray userData = settings.getUserData();
  interface.calibrate(userData);
}

void Controller::calibrateSystemOffset()
{
  QByteArray userData = settings.getUserData();
  interface.calibrateSystemOffset(userData);
}

//Called on application close
void Controller::close()
{
  interface.close();
  settings.save();
}

//Forward the data
void Controller::receivedData(const char *pData, const int length, const bool complete)
{
  //Only use the complete data, no broken images
  if (complete == false)
  {
    return;
  }

  DataHeader dataHeader(pData, length);

  emit receivedMeasurementData(pData, dataHeader);
}

//Forward connected event
void Controller::cmdConnectionConnected()
{

  emit connected();

}

//Forward disconnected event
void Controller::cmdConnectionDisconnected()
{
  emit disconnected();
}

//Forward updateFps event
void Controller::onFpsUpdate(const unsigned int fps)
{
  emit updateFps(fps);
}
