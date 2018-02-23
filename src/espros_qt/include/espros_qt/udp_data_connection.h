#ifndef UDP_DATA_CONNECTION_H
#define UDP_DATA_CONNECTION_H

#include <stdint.h>
#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QThread>
#include "data_connection.h"

class UdpDataConnection: public DataConnection
{
  Q_OBJECT

  public:
    UdpDataConnection(QObject *parent = 0);
    ~UdpDataConnection();
    void setPort(const uint16_t port);

  public slots:
    void startRunning();
    void stopRunning();

  private slots:
    void processPendingDatagrams();

  private:
    uint16_t getHeaderUint16(QByteArray data, const int offset);
    uint32_t getHeaderUint32(QByteArray data, const int offset);
    uint16_t getHeaderUint16(char *pData, const int offset);
    uint32_t getHeaderUint32(char *pData, const int offset);

    uint16_t port;
    QUdpSocket *socket = nullptr;
    uint16_t actualNumber;
    unsigned int receivedBytes;
    char rxBuffer[10000000];
};

#endif // UDP_DATA_CONNECTION_H
