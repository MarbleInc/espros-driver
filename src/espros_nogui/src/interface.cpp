#include "espros_nogui/interface.h"
#include "espros_nogui/tcp_cmd_connection.h"
#include "espros_nogui/udp_data_connection.h"
#include "espros_nogui/data_header.h"
#include <QByteArray>
#include <QDebug>
#include <QtCore>

//commands GUI-->TOFCAM
typedef enum
{
  TOFCAM_COMMAND_SET_INT_TIMES = 1,
  TOFCAM_COMMAND_GET_DISTANCE_AMPLITUDE = 2,
  TOFCAM_COMMAND_GET_DISTANCE = 3,
  TOFCAM_COMMAND_GET_GRAYSCALE = 5,
  TOFCAM_COMMAND_STOP_STREAM = 6,
  TOFCAM_COMMAND_SET_OFFSET = 20,
  TOFCAM_COMMAND_SET_MIN_AMPLITUDE = 21,
  TOFCAM_COMMAND_CALIBRATE = 30,
  TOFCAM_COMMAND_CALIBRATE_SYSTEM_OFFSET = 31
}tofcamCommand_e;

//answers TOFCAM-->GUI
typedef enum
{
  TOFCAM_ANSWER_ACK = 0,
  TOFCAM_ANSWER_ERROR = 1
}tofcamAnswer_e;

Interface::Interface(QObject *parent  __attribute__((unused)))
{
  numMeasurements = 0;

  udpThread = new InterfaceThread();
  tcpThread = new InterfaceThread();
  timerThread = new InterfaceThread();

  dataConnection = new UdpDataConnection(this);
  connect(dataConnection, &DataConnection::receivedData, this, &Interface::receivedData);
  dataConnection->setPort(45454);

  cmdConnection = new TcpCmdConnection();
  cmdConnection->setConnectionParameters("10.10.31.180", 50660);
  connect(cmdConnection, &TcpCmdConnection::receivedAnswer, this, &Interface::receivedAnswer);
  connect(cmdConnection, &TcpCmdConnection::connected, this, &Interface::cmdConnectionConnected);
  connect(cmdConnection, &TcpCmdConnection::disconnected, this, &Interface::cmdConnectionDisconnected);

  waitAck = false;

  cmdConnection->moveToThread(tcpThread);
  cmdConnection->connect(tcpThread, &QThread::started, cmdConnection, &CmdConnection::startRunning);
  cmdConnection->connect(tcpThread, &QThread::finished, cmdConnection, &CmdConnection::stopRunning);
  tcpThread->start();

  dataConnection->moveToThread(udpThread);
  dataConnection->connect(udpThread, &QThread::started, dataConnection, &DataConnection::startRunning);
  dataConnection->connect(udpThread, &QThread::finished, dataConnection, &DataConnection::stopRunning);
  udpThread->start();

  fpsTimer = new QTimer(0); //Parent must be null
  fpsTimer->setInterval(1000);
  fpsTimer->start();
  fpsTimer->moveToThread(timerThread);
  connect(fpsTimer, SIGNAL(timeout()), this, SLOT(onUpdateFps()));
  timerThread->start();
}

Interface::~Interface()
{
  close();
  delete cmdConnection;
  delete dataConnection;
  delete udpThread;
}

void Interface::close()
{
  stopStream();
  udpThread->stopRunning();
  tcpThread->stopRunning();
  timerThread->stopRunning();

  waitAck = true;
  QTimer timer;
  timer.setSingleShot(true);
  timer.start(1000);

  while(waitAck && (timer.isActive()))
  {
    QCoreApplication::processEvents();
  }
  cmdConnection->close();
}

void Interface::insertValue(QByteArray &output, const uint16_t value)
{
  output.append(value >> 8);
  output.append(value & 0xFF);
}

void Interface::insertValue(QByteArray &output, const int16_t value)
{
  output.append(value >> 8);
  output.append(value & 0xFF);
}

uint8_t Interface::boolToUint8(const bool value)
{
  if (value)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void Interface::sendCommand(QByteArray &dataToSend, const QByteArray &userData)
{
  //If present, add user data
  if (userData != nullptr)
  {
    dataToSend.append(userData);
  }

  cmdConnection->sendCommand(dataToSend);
}


void Interface::requestDistanceAmplitude(const bool doStream, const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_GET_DISTANCE_AMPLITUDE;
  uint8_t stream = boolToUint8(doStream);

  //Insert the 16Bit command
  insertValue(outputData, command);

  //Insert the stream flag
  outputData.insert(2, stream);

  sendCommand(outputData, userData);
}

void Interface::requestDistance(const bool doStream, const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_GET_DISTANCE;
  uint8_t stream = boolToUint8(doStream);

  //Insert the 16Bit command
  insertValue(outputData, command);

  //Insert the stream flag
  outputData.insert(2, stream);

  sendCommand(outputData, userData);
}

void Interface::requestGrayscale(const bool doStream, const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_GET_GRAYSCALE;
  uint8_t stream = boolToUint8(doStream);

  //Insert the 16Bit command
  insertValue(outputData, command);

  //Insert the stream flag
  outputData.insert(2, stream);

  sendCommand(outputData, userData);
}

void Interface::setIntegrationTimes(const uint16_t integrationTime3d[3], const uint16_t integrationTimeGrayscale, const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_SET_INT_TIMES;

  //Insert the 16Bit command
  insertValue(outputData, command);

  //Insert the 3 integration times for 3D
  for (int i = 0; i < 3; i++)
  {
    insertValue(outputData, integrationTime3d[i]);
  }

  //Insert the integration time for grayscale
  insertValue(outputData, integrationTimeGrayscale);

  sendCommand(outputData, userData);
}

void Interface::setOffset(const int16_t offset, const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_SET_OFFSET;

  //Insert the 16Bit command
  insertValue(outputData, command);

  //Insert the offset
  insertValue(outputData, offset);

  sendCommand(outputData, userData);
}

void Interface::setMinAmplitude(const uint16_t minAmplitude, const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_SET_MIN_AMPLITUDE;

  //Insert the 16Bit command
  insertValue(outputData, command);

  //Insert the min Amplitude
  insertValue(outputData, minAmplitude);

  sendCommand(outputData, userData);
}

void Interface::stopStream(const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_STOP_STREAM;

  //Insert the 16Bit command
  insertValue(outputData, command);

  sendCommand(outputData, userData);
}

void Interface::calibrate(const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_CALIBRATE;

  //Insert the 16Bit command
  insertValue(outputData, command);

  sendCommand(outputData, userData);
}

void Interface::calibrateSystemOffset(const QByteArray &userData)
{
  QByteArray outputData;

  uint16_t command = TOFCAM_COMMAND_CALIBRATE_SYSTEM_OFFSET;

  //Insert the 16Bit command
  insertValue(outputData, command);

  sendCommand(outputData, userData);
}

void Interface::receivedAnswer(QByteArray data)
{
  uint8_t answer = data.at(0);
  switch(answer)
  {
    case TOFCAM_ANSWER_ACK:
      emit receivedAck();
      waitAck = false;
      break;
    case TOFCAM_ANSWER_ERROR:
      {
        uint8_t errorNumber = data.at(1);
        emit receivedError(errorNumber);
        qDebug() << "Received Error: " << errorNumber;
      }
      break;
    default:
      qDebug() << "Unknown answer";
      break;
  }
}

void Interface::onUpdateFps()
{
  emit updateFps(numMeasurements);
  numMeasurements = 0;
}

void Interface::receivedData(const char *pData, const int length, bool complete)
{
  if (complete)
  {
    numMeasurements++;
  }
  emit receivedMeasurementData(pData, length, complete);
}

void Interface::cmdConnectionConnected()
{
  emit connected();
}

void Interface::cmdConnectionDisconnected()
{
  emit disconnected();
}
