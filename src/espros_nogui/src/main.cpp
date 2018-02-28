/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
#include "espros_nogui/controller.h"
#include "espros_nogui/settings.h"
#include "espros_nogui/interface.h"
#include "espros_nogui/qnode.hpp"

#include <QCoreApplication>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  /*********************
  ** Qt
  **********************/
  QCoreApplication app(argc, argv);

  //Passing 3rd arg may fix x server dependency
  //QCoreApplication app(argc, argv, false);

  Interface interface;
  Settings settings;
  Controller controller(settings, interface);

  QNode qnode(argc, argv, controller);
  qnode.setSettings(&settings);

  if (!qnode.init()) {
    std::cout << "QNode init failed." << std::endl;
    exit(0);
  }

  QObject::connect(&qnode, SIGNAL(rosShutdown()), &app, SLOT(quit()));

  int result = app.exec();

	return result;
}
