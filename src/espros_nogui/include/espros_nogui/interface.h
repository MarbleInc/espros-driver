#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdint.h>
#include <QObject>
#include <QByteArray>
#include <QTimer>
#include "interface_thread.h"
#include "cmd_connection.h"
#include "data_connection.h"

class Interface: public QObject
{
  Q_OBJECT

  public:
    explicit Interface(QObject *parent = 0);
    ~Interface();
    void requestDistanceAmplitude(const bool doStream, const QByteArray &userData = nullptr);
    void requestDistance(const bool doStream, const QByteArray &userData = nullptr);
    void requestGrayscale(const bool doStream, const QByteArray &userData = nullptr);
    void setIntegrationTimes(const uint16_t integrationTime3d[3], const uint16_t integrationTimeGrayscale, const QByteArray &userData = nullptr);
    void setOffset(const int16_t offset, const QByteArray &userData = nullptr);
    void setMinAmplitude(const uint16_t minAmplitude, const QByteArray &userData = nullptr);
    void stopStream(const QByteArray &userData = nullptr);
    void calibrate(const QByteArray &userData = nullptr);
    void calibrateSystemOffset(const QByteArray &userData = nullptr);
    void close();

  signals:
    void receivedAck();
    void receivedError(const int errorCode);
    void receivedMeasurementData(const char *pData, const int length, bool complete);
    void connected();
    void disconnected();
    void stopRunning();
    void updateFps(const unsigned int fps);

  private slots:
    void receivedAnswer(QByteArray data);
    void receivedData(const char *pData, const int length, bool complete);
    void cmdConnectionConnected();
    void cmdConnectionDisconnected();
    void onUpdateFps();

  private:
    void insertValue(QByteArray &output, const uint16_t value);
    void insertValue(QByteArray &output, const int16_t value);
    uint8_t boolToUint8(const bool value);
    void sendCommand(QByteArray &dataToSend, const QByteArray &userData);

    InterfaceThread *timerThread;
    InterfaceThread *udpThread;
    InterfaceThread *tcpThread;
    CmdConnection *cmdConnection;
    DataConnection *dataConnection;
    bool waitAck;
    QTimer *fpsTimer;
    unsigned int numMeasurements;


};

#endif // INTERFACE_H
