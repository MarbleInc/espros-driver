#include "espros_nogui/image_colorizer.h"
#include <QDebug>

#define NUM_COLORS  6000

#define LOW_AMPLITUDE 16001
#define SATURATION    16002
#define ADC_OVERFLOW  16003


ImageColorizer::ImageColorizer(QObject *)
{
    int numSteps = NUM_COLORS;
    unsigned char red, green, blue;

    colorVector = QVector<Color>();

    for(int i=0;  i< numSteps; i++){
        createColorMap(numSteps, i, red, green, blue);
        colorVector << Color{red, green, blue};
    }
}

ImageColorizer::~ImageColorizer() {
    qDebug() << "***ImageColorizer destroyed";
}

void ImageColorizer::setRange(int start, int stop){
    begin = start;
    end = stop;
    indexFactorColor = NUM_COLORS / (double)(stop - start);
    indexFactorBw = 255.0 / (stop - start);
}

Color ImageColorizer::getColor(int value, ColorSpace colorspace){
    if (colorspace == RGB){
        return getCol(value);
    } else {
      return getBw(value);
    }
}

Color ImageColorizer::getCol(int value){
    if (value == SATURATION){
           return Color{255,0,128};
    }
    else if (value == ADC_OVERFLOW){
           return Color{169,14,255};
    }
    else if(value == LOW_AMPLITUDE){
            return Color{0, 0, 0};
    }
    else if(value == 0){
        return colorVector.at(0);
    }
    value -= begin;
    if (value < 0){
        return Color{0, 0, 0};
    }
    else if (value > end){
        return Color{0, 0, 0};
    }
    return colorVector.at(colorVector.size() - (value*indexFactorColor));
}

Color ImageColorizer::getBw(int value){
    if (value == SATURATION){
       return Color{255,0,128};
    } else if (value > end){
        return Color{255, 255, 255};
    } else if (value < begin){
        return Color{0, 0, 0};
    } else{
        int color = value * indexFactorBw;
        return Color{color, color, color};
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
