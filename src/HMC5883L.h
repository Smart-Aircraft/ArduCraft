#include <Wire.h>
#include "Arduino.h"
    
#ifndef HMC5883L_h
#define HMC5883L_h

#define HMC5883L_I2C_ADDRESS       0x1E //001 1110b(0x3C>>1), HMC5883的7位i2c地址  
#define MAGNETIC_DECLINATION       -5.5 //所在地磁偏角
#define ANG_TO_RAD                 0.0174533

class HMC5883L
{
  public:

    void begin();

    double getHeading(double newRoll, double newPitch);  //算方位角

  private:
    int xMax;//xMax: 415  zMin: -344   yMax: 136  Ymin: -760 zMax: 406  zMin: -424
    int xMin;
    int yMax;
    int yMin;
    int zMax;
    int zMin;

    double xNorm;//经过Z补偿过后的值
    double yNorm;
    double xOut;//经过offset调整过的值
    double yOut;
    double zOut;
    double headingRadians;
    double headingDegrees;

    int x;
    int y;
    int z;

    double Xs;
    double Ys;
    double Zs;
    double Xb;
    double Yb;
    double Zb;
    double offsetX;
    double offsetY;
    double offsetZ;

    void getRawData();//得到初始数据
};

#endif
