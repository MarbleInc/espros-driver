#include "espros_nogui/settings.h"
#include "QStandardPaths"

Settings::Settings()
{
  QString path = QStandardPaths::writableLocation(QStandardPaths::ConfigLocation);
  setupSettings = new QSettings(path + "/TOFCAMSettings.ini", QSettings::IniFormat);
  restore();
}

void Settings::setRange(const unsigned int range)
{
  this->range = range;
  //no signal to emit
}
unsigned int Settings::getRange() const
{
  return range;
}

void Settings::setIntegrationTime0(const unsigned int integrationTime)
{
  this->integrationTime0 = integrationTime;
  emit integrationTimeChanged();
}
unsigned int Settings::getIntegrationTime0() const
{
  return integrationTime0;
}

void Settings::setIntegrationTime1(const unsigned int integrationTime)
{
  this->integrationTime1 = integrationTime;
  emit integrationTimeChanged();
}
unsigned int Settings::getIntegrationTime1() const
{
  return integrationTime1;
}

void Settings::setIntegrationTime2(const unsigned int integrationTime)
{
  this->integrationTime2 = integrationTime;
  emit integrationTimeChanged();
}
unsigned int Settings::getIntegrationTime2() const
{
  return integrationTime2;
}

void Settings::setIntegrationTimeGrayscale(const unsigned int integrationTime)
{
  this->integrationTimeGrayscale = integrationTime;
  emit integrationTimeChanged();
}
unsigned int Settings::getIntegrationTimeGrayscale() const
{
  return integrationTimeGrayscale;
}

void Settings::setOffset(const int offset)
{
  this->offset = offset;
  emit offsetChanged();
}
int Settings::getOffset() const
{
  return offset;
}

void Settings::setMinAmplitude(const unsigned int amplitude)
{
  this->minAmplitude = amplitude;
  emit minAmplitudeChanged();
}
unsigned int Settings::getMinAmplitude() const
{
  return minAmplitude;
}

void Settings::setUserData(const QByteArray userData)
{
  this->userData = userData;
  //no signal to emit
}
QByteArray Settings::getUserData() const
{
  return userData;
}

void Settings::restore()
{
  integrationTime0 = setupSettings->value("IntTime0", 500).toInt();
  integrationTime1 = setupSettings->value("IntTime1", 0).toInt();
  integrationTime2 = setupSettings->value("IntTime2", 0).toInt();
  integrationTimeGrayscale = setupSettings->value("IntTimeGrayscale", 100).toInt();
  offset =  setupSettings->value("Offset", 0).toInt();
  minAmplitude = setupSettings->value("MinAmplitude", 75).toInt();
  range = setupSettings->value("Range", 4000).toInt();
  userData = setupSettings->value("UserData").toByteArray();
}

void Settings::save()
{
  setupSettings->setValue("IntTime0", QVariant::fromValue(integrationTime0));
  setupSettings->setValue("IntTime1", QVariant::fromValue(integrationTime1));
  setupSettings->setValue("IntTimeGrayscale", QVariant::fromValue(integrationTimeGrayscale));
  setupSettings->setValue("Offset", QVariant::fromValue(offset));
  setupSettings->setValue("MinAmplitude", QVariant::fromValue(minAmplitude));
  setupSettings->setValue("Range", QVariant::fromValue(range));
  setupSettings->setValue("UserData", QVariant::fromValue(userData));
}
