#include "HMC5883L.h"

void HMC5883L::begin()
{
  xMax = 437;
  xMin = -549;
  yMax = 689;
  yMin = -347;
  zMax = 409;
  zMin = -594;
  
  Xs = 1;
  Ys = double((xMax-xMin))/double((yMax-yMin));
  Zs = double((xMax-xMin))/double((zMax-zMin));
  Xb = Xs*(0.5*(xMax-xMin)-xMax);
  Yb = Ys*(0.5*(yMax-yMin)-yMax);
  Zb = Xs*(0.5*(zMax-zMin)-zMax);
  offsetX = double((xMax+xMin)/2);
  offsetY = double((yMax+yMin)/2);
  offsetZ = double((zMax+zMin)/2);

  Wire.begin();  
    
  //设置HMC5883模式  
  Wire.beginTransmission(HMC5883L_I2C_ADDRESS); //开始通信  
  Wire.write(0x00); //选择配置寄存器A  
  Wire.write(0x70); //0111 0000b，具体配置见数据手册  
  Wire.endTransmission();  
    
  Wire.beginTransmission(HMC5883L_I2C_ADDRESS);  
  Wire.write(0x02); //选择模式寄存器  
  Wire.write(0x00); //连续测量模式:0x00,单一测量模式:0x01  持续测量模式 频率较低 不容易间断  单一测量模式 频率高 容易断掉停止
  Wire.endTransmission();     
}

void HMC5883L::getRawData()  
{
  Wire.beginTransmission(HMC5883L_I2C_ADDRESS);  
  Wire.write(0x03); //从寄存器3开始读数据  
  Wire.endTransmission();  
  //每轴的数据都是16位的  
  Wire.requestFrom(HMC5883L_I2C_ADDRESS, 6);  
  if(Wire.available() >= 6){  
    x = Wire.read()<<8; //X msb，X轴高8位  
    x |= Wire.read(); //X lsb，X轴低8位  
    z = Wire.read()<<8; //Z msb  
    z |= Wire.read(); //Z lsb  
    y = Wire.read()<<8; //Y msb  
    y |= Wire.read(); //Y lsb  
  }
}

double HMC5883L::getHeading(double newRoll,double newPitch)  //算方位角
{  
  getRawData();

  xOut=Xs*(double(x)-offsetX);
  yOut=Ys*(double(y)-offsetY);
  zOut=Zs*(double(z)-offsetZ);

  xNorm=(xOut)*cos(newPitch*ANG_TO_RAD)+(yOut)*sin(newPitch*ANG_TO_RAD)*sin(newRoll*ANG_TO_RAD)-(zOut)*cos(newRoll*ANG_TO_RAD)*sin(newPitch*ANG_TO_RAD);
  yNorm=(yOut)*cos(newRoll*ANG_TO_RAD)+(zOut)*sin(newRoll*ANG_TO_RAD);
  
  headingRadians = atan2(yNorm,xNorm); 
  //保证数据在0-2*PI之间  
  if(headingRadians < 0)  
    headingRadians += 2*PI;  
    
  headingDegrees = headingRadians * 180/M_PI;  
  headingDegrees += MAGNETIC_DECLINATION; //磁偏角  
    
  //保证数据在0-360之间
  if(headingDegrees > 360)  
    headingDegrees -= 360;  
  else if(headingDegrees < 0)  
    headingDegrees += 360;  
  return headingDegrees;  
}

