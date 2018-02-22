#include "../include/espros_qt/image_colorizer.h"
#include <QColor>
#include <QDebug>
#include <QRgb>

#define NUM_COLORS  6000

#define LOW_AMPLITUDE 16001
#define SATURATION    16002
#define ADC_OVERFLOW  16003


ImageColorizer::ImageColorizer(QObject *) :
legendRedToBlue(1, NUM_COLORS,QImage::Format_ARGB32_Premultiplied),
legendBlueToRed(1, NUM_COLORS,QImage::Format_ARGB32_Premultiplied),
legendGrayscale(1, 255, QImage::Format_ARGB32_Premultiplied)
{
    int numSteps = NUM_COLORS;
    unsigned char red, green, blue;

    colorVector = QVector<QColor>();

    for(int i=0;  i< numSteps; i++){
        createColorMap(numSteps, i, red, green, blue);
        colorVector << QColor(red, green, blue);
    }

    for (int i = 0; i<colorVector.size(); i++){
        legendBlueToRed.setPixel(0, colorVector.size()-i-1, colorVector.at(i).rgb());
        legendRedToBlue.setPixel(0, i, colorVector.at(i).rgb());
    }
    for (int i = 0; i<255; i++){
        legendGrayscale.setPixel(0, 255-1-i, qRgb(i,i,i));
    }
}

ImageColorizer::~ImageColorizer() {
    qDebug() << "***ImageColorizer destroyed";
}

void ImageColorizer::setRange(int start, int stop){
    begin = start;
    end = stop;
    indexFactorColor = NUM_COLORS / (double)(stop - start);

    //qDebug()<<"ImageColorizer::setRange stop start = " << stop << start << "indexFactorColor= "<< indexFactorColor;

    indexFactorBw = 255.0 / (stop - start);
}

QColor ImageColorizer::getColor(int value, ColorSpace colorspace){
    if (colorspace == REDTOBLUE){
        return getCol(value);
    } else if (colorspace == BLUETORED){
        return getColInverted(value);
    }
    return getBw(value);
}

QColor ImageColorizer::getCol(int value){
    if (value == SATURATION){
           return QColor(255,0,128);
    }
    else if (value == ADC_OVERFLOW){
           return QColor(169,14,255);
    }
    else if(value == LOW_AMPLITUDE){
            return QColor(0, 0, 0);
    }
    else if(value == 0){
        return colorVector.at(0);
    }
    value -= begin;
    if (value < 0){
        return QColor(0, 0, 0);
    }
    else if (value > end){
        return QColor(0, 0, 0);
    }
    return colorVector.at(colorVector.size() - (value*indexFactorColor));
}

QColor ImageColorizer::getColInverted(int value){
    if (value == SATURATION)
    {
      return QColor(255,0,128);
    }
    else if (value == ADC_OVERFLOW)
    {
      return QColor(169,14,255);
    }
    else if(value == 0)
    {
        return colorVector.at(0);
    }
    value -= begin;
    if (value < 0){
        return QColor(0, 0, 0);
    }
    else if (value > end){
        return QColor(0, 0, 0);
    }
    return colorVector.at(value * indexFactorColor -1);
}

QColor ImageColorizer::getBw(int value){
    if (value == SATURATION){
       return QColor(255,0,128);
    } else if (value > end){
        return QColor(255, 255, 255);
    } else if (value < begin){
        return QColor(0, 0, 0);
    } else{
        int color = value * indexFactorBw;
        return QColor(color, color, color);
    }
}


QImage &ImageColorizer::getLegend(ColorSpace colorspace){
    switch(colorspace){
    case REDTOBLUE:
        return legendRedToBlue;
    case BLUETORED:
        return legendBlueToRed;
    case GRAYSCALE:
    default:
        return legendGrayscale;
    }
}


double ImageColorizer::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}


void ImageColorizer::createColorMap(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue){

    /*double B0 = -0.125;
    double B1 = B0 + 0.25;
    double B2 = B1 + 0.25;
    double B3 = B2 + 0.25;

    double G0 = B1;
    double G1 = G0 + 0.25;
    double G2 = G1 + 0.25;
    double G3 = G2 + 0.25;

    double R0 = B2;
    double R1 = R0 + 0.25;
    double R2 = R1 + 0.25;
    double R3 = R2 + 0.25;*/

    double k = 1;
    double B0 = -0.125 * k - 0.25;
    double B1 = B0 + 0.25 * k;
    double B2 = B1 + 0.25 * k;
    double B3 = B2 + 0.25 * k;

    double G0 = B1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = B2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;

    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= B0 && i < B1 ){
        blue = interpolate(i, B0, 0, B1, 255);
    } else if((i >= B1)&&(i < B2)){
        blue = 255;
    } else if((i >= B2)&&(i < B3)) {
        blue = interpolate(i, B2, 255, B3, 0);
    } else{
        blue = 0;
    }
}
