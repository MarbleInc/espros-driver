#ifndef SETTINGS_H
#define SETTINGS_H

#include <QSettings>
#include <QObject>
#include <QByteArray>


class Settings: public QObject
{

  Q_OBJECT

public:
    Settings();

    unsigned int getRange() const;
    unsigned int getIntegrationTime0() const;
    unsigned int getIntegrationTime1() const;
    unsigned int getIntegrationTime2() const;
    unsigned int getIntegrationTimeGrayscale() const;
    int getOffset() const;
    unsigned int getMinAmplitude() const;
    QByteArray getUserData() const;

    void restore();
    void save();

public slots:
    void setMinAmplitude(const unsigned int amplitude);
    void setIntegrationTime0(const unsigned int integrationTime);
    void setIntegrationTime1(const unsigned int integrationTime);
    void setIntegrationTime2(const unsigned int integrationTime);
    void setIntegrationTimeGrayscale(const unsigned int integrationTime);
    void setRange(const unsigned int range);
    void setOffset(const int offset);
    void setUserData(const QByteArray userData);

signals:
    void offsetChanged();
    void minAmplitudeChanged();
    void integrationTimeChanged();

private:
  unsigned int range;
  unsigned int integrationTime0;
  unsigned int integrationTime1;
  unsigned int integrationTime2;
  unsigned int integrationTimeGrayscale;
  int offset;
  unsigned int minAmplitude;
  QByteArray userData;
  QSettings *setupSettings;
};

#endif // SETTINGS_H
