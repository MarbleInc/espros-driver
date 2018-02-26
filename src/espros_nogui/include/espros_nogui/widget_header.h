#ifndef WIDGET_HEADER_H
#define WIDGET_HEADER_H

#include <QWidget>
#include "data_header.h"

namespace Ui {
class WidgetHeader;
}

class WidgetHeader: public QWidget
{
   Q_OBJECT

public:
    explicit WidgetHeader(QWidget *parent = 0);
    ~WidgetHeader();
    void showHeaderData(DataHeader &dataHeader);

  private:
    Ui::WidgetHeader *ui;
};

#endif // WIDGET_HEADER_H
