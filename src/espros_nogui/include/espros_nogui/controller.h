#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include "stdint.h"
#include "interface.h"
#include "settings.h"
#include "data_header.h"

class Controller: public QObject
{
Q_OBJECT

public:
    Controller(Settings &settings, Interface &interface);
    void sendAllSettingsToCamera();
    void requestDistance(const bool isStream);
    void requestAmplitude(const bool isStream);
    void requestGrayscale(const bool isStream);
    void requestDistanceAmplitude(const bool isStream);
    void stopStream();
    void calibrate();
    void calibrateSystemOffset();
    void close();

signals:
    void receivedMeasurementData(const char *pData, DataHeader &dataHeader);
    void updateFps(const unsigned int fps);
    void connected();
    void disconnected();

private slots:
    void receivedData(const char *pData, const int length, const bool complete);
    void onFpsUpdate(const unsigned int fps);
    void cmdConnectionConnected();
    void cmdConnectionDisconnected();

private:
    void onOffsetChanged();
    void onIntegrationTimesChanged();
    void onMinAmplitudeChanged();

    Settings &settings;
    Interface &interface;
};

#endif // CONTROLLER_H
