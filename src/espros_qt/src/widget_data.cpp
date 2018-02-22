#include "espros_qt/widget_data.h"
#include "ui_widget_data.h"
#include "espros_qt/image_colorizer.h"
#include "math.h"
#include <QImage>

const double maxAmplitudeValue = 2600.0;
const unsigned int maxValidValue = 15000;


//Data TOFCAM-->GUI
typedef enum
{
  TOFCAM_DATA_DISTANCE_AMPLITUDE = 0,
  TOFCAM_DATA_DISTANCE = 1,
  TOFCAM_DATA_AMPLITUDE = 2,
  TOFCAM_DATA_GRAYSCALE = 3
}tofCamData_e;

WidgetData::WidgetData(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetData)
{
  ui->setupUi(this);
}

WidgetData::~WidgetData()
{
  delete ui;
}

void WidgetData::setSettings(const Settings *settings)
{
  this->settings = (Settings *)settings;
}

unsigned int WidgetData::calcLog(const unsigned int value)
{
  double multiplier = maxAmplitudeValue / log10(maxAmplitudeValue);

  //get zero
  if (value == 0)
  {
    return 0;
  }

  //get special values, for example ADC_OVERFLOW
  if (value > maxValidValue)
  {
    return value;
  }

  double newValue = log10((double)value) * multiplier;
  return (unsigned int)newValue;
}

void WidgetData::updateImage(QLabel &label, const QImage &image)
{
  QImage newImage = image.mirrored(ui->checkBoxMirrorHorizontal->isChecked(), ui->checkBoxMirrorVertical->isChecked());

  QPixmap pixmap = QPixmap::fromImage(newImage);

  label.setPixmap(pixmap.scaled(label.size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  label.repaint();
}

void WidgetData::showDistance(const char *pData, DataHeader &dataHeader)
{
  QImage imageDistance(dataHeader.width, dataHeader.height, QImage::Format_RGB888);

  imageColorizerDistance.setRange(0, settings->getRange());

  int index = 0;
  for (int y = 0; y < dataHeader.height; y++)
  {
    for (int x = 0; x < dataHeader.width; x++)
    {
      //First get the values of the distance
      uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
      uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

      //Combint to a value
      unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;

      //Colorize the pixel
      imageDistance.setPixel(x, y, imageColorizerDistance.getColor(pixelDistance, ImageColorizer::REDTOBLUE).rgb());
      index++;
    }
  }

  updateImage(*ui->labelLeftImage, imageDistance);
  ui->labelRightImage->clear();
}

void WidgetData::showDistanceAmplitude(const char *pData, DataHeader &dataHeader)
{
  QImage imageDistance(dataHeader.width, dataHeader.height, QImage::Format_RGB888);
  QImage imageAmplitude(dataHeader.width, dataHeader.height, QImage::Format_RGB888);

  imageColorizerDistance.setRange(0, settings->getRange());
  imageColorizerAmplitude.setRange(0, int(maxAmplitudeValue));

  int index = 0;
  for (int y = 0; y < dataHeader.height; y++)
  {
    for (int x = 0; x < dataHeader.width; x++)
    {
      //First get the values of the distance
      uint8_t distanceMsb = (uint8_t) pData[4*index+1+dataHeader.offset];
      uint8_t distanceLsb = (uint8_t) pData[4*index+0+dataHeader.offset];

      //Then get the values of the amplitude
      uint8_t amplitudeMsb = (uint8_t) pData[4*index+3+dataHeader.offset];
      uint8_t amplitudeLsb = (uint8_t) pData[4*index+2+dataHeader.offset];

      //Combine to a value
      unsigned int pixelDistance = (distanceMsb << 8) + distanceLsb;
      unsigned int pixelAmplitude = (amplitudeMsb << 8) + amplitudeLsb;

      ImageColorizer::ColorSpace colorSpace = ImageColorizer::BLUETORED;

      //scale logarithmic only if grayscale amplitude is selected
      if (ui->checkBoxAmplitudeGrayscale->isChecked())
      {
        pixelAmplitude = calcLog(pixelAmplitude);
        colorSpace = ImageColorizer::GRAYSCALE;
      }

      //Colorize the pixel
      imageDistance.setPixel(x, y, imageColorizerDistance.getColor(pixelDistance, ImageColorizer::REDTOBLUE).rgb());

      imageAmplitude.setPixel(x, y, imageColorizerAmplitude.getColor(pixelAmplitude, colorSpace).rgb());
      index++;
    }
  }

  updateImage(*ui->labelLeftImage, imageDistance);
  updateImage(*ui->labelRightImage, imageAmplitude);
}

void WidgetData::showGrayscale(const char *pData, DataHeader &dataHeader)
{
  QImage imageGrayscale(dataHeader.width, dataHeader.height, QImage::Format_RGB888);

  imageColorizerGrayscale.setRange(0, 1024);

  int index = 0;
  for (int y = 0; y < dataHeader.height; y++)
  {
    for (int x = 0; x < dataHeader.width; x++)
    {
      //First get the values of the distance
      uint8_t distanceMsb = (uint8_t) pData[2*index+1+dataHeader.offset];
      uint8_t distanceLsb = (uint8_t) pData[2*index+0+dataHeader.offset];

      //Combine to a value
      unsigned int pixelGrayscale = (distanceMsb << 8) + distanceLsb;

      //Colorize the pixel
      imageGrayscale.setPixel(x, y, imageColorizerDistance.getColor(pixelGrayscale, ImageColorizer::GRAYSCALE).rgb());
      index++;
    }
  }

  updateImage(*ui->labelLeftImage, imageGrayscale);
  ui->labelRightImage->clear();
}

void WidgetData::showData(const char *pData, DataHeader &dataHeader)
{
  switch(dataHeader.dataType)
  {
    case TOFCAM_DATA_DISTANCE_AMPLITUDE:
      showDistanceAmplitude(pData, dataHeader);
      break;
    case TOFCAM_DATA_DISTANCE:
      showDistance(pData, dataHeader);
      break;
    case TOFCAM_DATA_GRAYSCALE:
      showGrayscale(pData, dataHeader);
      break;
    default:
      break;
  }
}
