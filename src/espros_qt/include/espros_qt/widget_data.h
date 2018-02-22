#ifndef WIDGETDATA_H
#define WIDGETDATA_H

#include <QWidget>
#include "data_header.h"
#include "settings.h"
#include "image_colorizer.h"
#include <QLabel>

namespace Ui {
class WidgetData;
}

class WidgetData : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetData(QWidget *parent = 0);
    ~WidgetData();
    void setSettings(const Settings *settings);
    void showDistance(const char *pData, DataHeader &dataHeader);
    void showDistanceAmplitude(const char *pData, DataHeader &dataHeader);
    void showGrayscale(const char *pData, DataHeader &dataHeader);
    void showData(const char *pData, DataHeader &dataHeader);

private:
    unsigned int calcLog(const unsigned int value);
    void updateImage(QLabel &label, const QImage &image);

    Ui::WidgetData *ui;
    Settings *settings;
    ImageColorizer imageColorizerDistance;
    ImageColorizer imageColorizerAmplitude;
    ImageColorizer imageColorizerGrayscale;
};

#endif // WIDGETDATA_H
