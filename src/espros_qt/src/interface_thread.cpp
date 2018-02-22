#include "espros_qt/interface_thread.h"

InterfaceThread::InterfaceThread()
{
}

void InterfaceThread::run()
{
  QThread::run();
}

void InterfaceThread::stopRunning()
{
  quit();
  wait();
}
