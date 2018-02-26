#ifndef COMMANDCONNECTION_H
#define COMMANDCONNECTION_H

#include <QObject>
#include <stdint.h>


class CmdConnection: public QObject
{
  Q_OBJECT

  public:
    virtual ~CmdConnection(){}
    virtual bool setConnectionParameters(QString ip, uint16_t port) = 0;
    virtual void sendCommand(QByteArray dataToSend) = 0;
    virtual void close() = 0;

  public slots:
    virtual void startRunning() = 0;
    virtual void stopRunning() = 0;

    signals:
    void receivedAnswer(QByteArray data);
    void connected();
    void disconnected();
};

#endif // COMMANDCONNECTION_H
