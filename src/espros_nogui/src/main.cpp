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
#include "espros_nogui/mainwindow.h"
#include "espros_nogui/controller.h"
#include "espros_nogui/settings.h"
#include "espros_nogui/interface.h"

#include <QtGui>
#include <QApplication>

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

    MainWindow w(argc, argv, controller, settings);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
