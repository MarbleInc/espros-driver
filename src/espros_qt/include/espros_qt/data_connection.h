#ifndef DATA_CONNECTION_H
#define DATA_CONNECTION_H

#include <QObject>

class DataConnection: public QObject
{
  Q_OBJECT

  public:
    virtual ~DataConnection(){}
    virtual void setPort(const uint16_t port) = 0;

  public Q_SLOTS:
    virtual void startRunning() = 0;
    virtual void stopRunning() = 0;

  Q_SIGNALS:
    void receivedData(const char *pData, const int length, const bool complete);

};

#endif // CONNECTION_H
