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
#include "../include/espros_qt/mainwindow.h"
#include "../include/espros_qt/controller.h"
#include "../include/espros_qt/settings.h"
#include "../include/espros_qt/interface.h"

#include <QtGui>
#include <QApplication>
//#include "../include/espros_qt/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

    Interface interface;
    Settings settings;
    Controller controller(settings, interface);

    espros_qt::MainWindow w(controller, settings, argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
