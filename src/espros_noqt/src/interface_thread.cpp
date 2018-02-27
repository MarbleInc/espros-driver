#include "espros_noqt/interface_thread.h"

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
