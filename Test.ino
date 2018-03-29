#include <Servo.h>
#include "MPU6050.h"
#include "MS5611.h"

Servo servo;
MPU6050 Gryo;
MS5611 Baro;

float Roll;
float Pitch;
double realTemperature;
double realPressure;
double realAltitude;
int angle = 50;
int count = 0;

void setup() 
{ 
  Serial.begin(115200); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  Gryo.begin(); //初始化陀螺仪
  delay(100);
  Baro.begin();
  delay(100);
  //servo.attach(9);
  //servo.write(angle);
} 

void loop() {
  count ++;
  if (count > 30) {
    realTemperature = Baro.readTemperature();
    realPressure = Baro.readPressure();
    realAltitude = Baro.getAltitude(realPressure);
    count = 0;
  }

  Gryo.ReadAng();
  Roll = Gryo.GetAngRoll();
  Pitch = Gryo.GetAngPitch();

  //angle = 50 + (Roll / 180) * 100;
  //servo.write(angle);

  Serial.print("--------------------------------------------------------\n");
  Serial.print("温度： ");
  Serial.print(realTemperature);
  Serial.print("°C   大气压： ");
  Serial.print(realPressure/100);
  Serial.print(" hpa   当前海拔： ");
  Serial.print(realAltitude);
  Serial.println("m");
  Serial.print("Roll: ");
  Serial.print(Roll); 
  Serial.print("     Pitch: ");
  Serial.print(Pitch);
  Serial.print('\n');

  delay(20);

} 

