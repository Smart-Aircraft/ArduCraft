#include <Kalman.h>
#include <Wire.h>
#include <Math.h>

//float MPUAccData[3]; //ACC_x,y,z
float MPUAngData[2]; //Roll and Pitch

float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数
const int MPU = 0x68; //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量

const int nCalibTimes = 1000; //校准时读数的次数
int calibData[nValCnt]; //校准数据

unsigned long nLastTime = 0; //上一次读数的时间
float fLastRoll = 0.0f; //上一次滤波得到的Roll角
float fLastPitch = 0.0f; //上一次滤波得到的Pitch角
float fLastYaw = 0.0f;

volatile float exInt, eyInt, ezInt; // 误差积分
volatile float q0, q1, q2, q3; // 全局四元数

float gx,gy,gz;
float ex,ey,ez;
const float Ki=0.002f;
const float Kp=100.0f;

const float halfT = 0.01f;

float fRoll,fPitch,fYaw;

Kalman kalmanRoll; //Roll角滤波器
Kalman kalmanPitch; //Pitch角滤波器
//Kalman kalmanYaw;

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050设备  
  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
  q0 = 1.0f , q1 = 0.0f , q2 = 0.0f , q3 = 0.0f;
  exInt = 0.0 ,eyInt = 0.0, ezInt = 0.0;
}

void loop() {
  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  //测量单位化
  for(int i=0;i<3;i++){
    realVals[i]=realVals[i]/fNorm;
  }
  //估计方向的重力
  gx = 2*(q1*q3 - q0*q2);
  gy = 2*(q0*q1 + q2*q3);
  gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  //经叉乘积并求出误差
  ex = (realVals[1]*gz - realVals[2]*gy);
  ey = (realVals[2]*gx - realVals[0]*gz);
  ez = (realVals[0]*gy - realVals[1]*gx);
  //积分误差比例积分增益
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;
  // 调整后的陀螺仪测量
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
  //四元数更新
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
  // 四元数规范化
  fNorm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / fNorm;
  q1 = q1 / fNorm;
  q2 = q2 / fNorm;
  q3 = q3 / fNorm;
  //get the angles
  //fYaw = atan2(2*q1*q2+2*q0*q3, ????)*fRad2Deg;
  fPitch= asin(-2*q1*q3+2*q0*q2)*fRad2Deg;
  fRoll= atan2(2*q0*q1+2*q2*q3,-2*q1*q1-2*q2*q2+1)*fRad2Deg;

  
  /*float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }*/

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  //float fNewYaw = kalmanYaw.getAngle(fYaw, realVals[6], dt);
  //跟据滤波值计算角度速
  //float fRollRate = (fNewRoll - fLastRoll) / dt;
  //float fPitchRate = (fNewPitch - fLastPitch) / dt;
  //float fYawRate = (fNewYaw - fLastYaw) / dt;
 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  //fLastYaw = fNewYaw;
  //更新本次测的时间
  nLastTime = nCurTime;

  Serial.print("Roll:");
  Serial.print(fNewRoll); 
  Serial.print("\t Pitch:");
  Serial.print(fNewPitch);
  Serial.print('\n');
  
  /*
  //向串口打印输出Roll角和Pitch角以及Roll和pitch角加速度，运行时在Arduino的串口监视器中查看
  Serial.print("Roll:");
  Serial.print(fNewRoll); Serial.print('(');
  Serial.print(fRollRate); Serial.print("),\tPitch:");
  Serial.print(fNewPitch); Serial.print('(');
  Serial.print(fPitchRate); Serial.print(")\n");
  */
  /*Serial.print(fPitchRate); Serial.print("),\tYaw:");
  Serial.print(fNewYaw); Serial.print('(');
  Serial.print(fYawRate); Serial.print(")\n");
  */
  /*
  Serial.print("Acc_x:");
  Serial.print(realVals[0]);
  Serial.print("\tAcc_y:");
  Serial.print(realVals[1]);
  Serial.print("\tAcc_z:");
  Serial.print(realVals[2]);
  Serial.print('\n');
  */

  //save Acc_x,y,z and Roll,Pitch angles
  /*for (int i = 0; i < 3; ++i){
    MPUAccData[i]=realVals[i];
  }
  */
  
  MPUAngData[0]= fNewRoll;
  MPUAngData[1]= fNewPitch;
  
  delay(20);
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
}



//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 2; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f; //unit: g
  }
  pRealVals[2] = (float)(pReadout[2] - calibData[2]) / 16384.0f + 1;
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;  //250度/s
  }
}

//return MPUAllData
/*float GetAccX(){
  float AccX;
  AccX = MPUAccData[0];
  return AccX;
}
float GetAccY(){
  float AccY;
  AccY = MPUAccData[1];
  return AccY;
}
float GetAccZ(){
  float AccZ;
  AccZ = MPUAccData[2];
  return AccZ;
}
*/

float GetAngRoll(){
  float AngRoll;
  AngRoll = MPUAngData[0];
  return AngRoll;
}
float GetAngPitch(){
  float AngPitch;
  AngPitch = MPUAngData[1];
  return AngPitch;
}

