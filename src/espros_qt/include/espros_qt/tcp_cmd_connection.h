#ifndef TCP_CMD_CONNECTION_H
#define TCP_CMD_CONNECTION_H

#include <stdint.h>
#include "cmd_connection.h"
#include <QtNetwork/QTcpSocket>
#include <QTimer>
#include <QSemaphore>
#include <QQueue>
#include <QThread>

class TcpCmdConnection: public CmdConnection
{
  Q_OBJECT

  enum State
  {
    STATE_CONNECTING,
    STATE_UNCONNECTED,
    STATE_CONNECTED,
    STATE_CLOSING,
    STATE_WAIT_ACK
  };

  public:
    TcpCmdConnection(QObject *parent = 0);
    ~TcpCmdConnection();
    bool setConnectionParameters(QString ip, uint16_t port);
    void sendCommand(QByteArray dataToSend);
    void close();

  public Q_SLOTS:
    void startRunning();
    void stopRunning();

  private Q_SLOTS:
    void readTcpData();
    void TcpConnected();
    void TcpDisconnected();
    void timeout();
    void checkToSendData();
    void error(QAbstractSocket::SocketError error);

  Q_SIGNALS:
    void checkData();

  private:
    const unsigned int TIMER_INTERVAL = 1000;

    bool isValidData(QByteArray data);
    bool hasValidStartMarking(QByteArray data);
    bool hasValidEndMarking(QByteArray data);
    bool lengthIsCorrect(QByteArray data);
    int output(QTcpSocket &socket, QByteArray dataToSend);

    QByteArray receiverData;
    QQueue<QByteArray> queue;
    QTcpSocket *tcpSocket;
    QString destIp;
    uint16_t destPort;
    State state;
    QTimer *timeoutTimer;
};


#endif // TCP_CMD_CONNECTION_H
