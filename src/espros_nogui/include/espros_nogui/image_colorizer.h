#ifndef IMAGE_COLORIZER_H_
#define IMAGE_COLORIZER_H_

#include <QObject>
#include <QVector>

typedef struct Color {
  int r;
  int g;
  int b;
} Color;

class ImageColorizer : public QObject {
    Q_OBJECT
public:
    explicit ImageColorizer (QObject *parent = 0);
    void setRange(int start, int stop);
    enum ColorSpace {GRAYSCALE, RGB};
    Color getColor(int value, ColorSpace colorspace);
    ~ImageColorizer();

private:
    QVector<Color> colorVector;
    int begin;
    int end;
    double indexFactorColor;
    double indexFactorBw;
    Color getCol(int value);
    Color getBw(int value);
    void createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
    double interpolate( double x, double x0, double y0, double x1, double y1);
};

#endif // IMAGE_COLORIZER_H_
