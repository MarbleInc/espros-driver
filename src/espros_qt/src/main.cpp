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
#include "espros_qt/mainwindow.h"
#include "espros_qt/controller.h"
#include "espros_qt/settings.h"
#include "espros_qt/interface.h"
#include "espros_qt/qnode.hpp"

#include <QtGui>
#include <QApplication>
//#include "espros_qt/main_window.hpp"

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

    MainWindow w(controller, settings);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
