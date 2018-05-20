#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"
#include "PWM.h"

HMC5883L Compass;
MPU6050 Gryo;
MS5611 Baro;

double Roll;
double Pitch;
double realTemperature;
double realPressure;
double realAltitude;
double Heading;
int count = 0;

void setup() 
{ 
  Serial.begin(115200); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  delay(100);
  Gryo.begin(); //初始化陀螺仪
  delay(100);
  Baro.begin();
  delay(100);
  Baro.init();
  delay(100);
  Compass.begin();
} 

void loop() {

  Gryo.ReadAng();
  Roll = Gryo.GetAngRoll();
  Pitch = Gryo.GetAngPitch();

  Heading = Compass.getHeading(-Roll, -Pitch);

  if (count > 30) {
    realTemperature = Baro.readTemperature();
    realPressure = Baro.readPressure();
    realAltitude = Baro.getAltitude(realPressure);
    count = 0;
  }
  count ++;

  Serial.print("温度： ");
  Serial.print(realTemperature);
  Serial.print("     当前海拔： ");
  Serial.print(realAltitude);
  Serial.print("m");
  Serial.print("     Roll: ");
  Serial.print(Roll); 
  Serial.print("     Pitch: ");
  Serial.print(Pitch);
  Serial.print("     Heading: ");
  Serial.println(Heading);
  delay(20);

}
