#include "MPU6050.h"

void MPU6050::begin() {
  WriteMPUReg(0x6B, 0); //启动MPU6050设备  
  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
  q0 = 1.0f , q1 = 0.0f , q2 = 0.0f , q3 = 0.0f;
  exInt = 0.0 ,eyInt = 0.0, ezInt = 0.0;
}

void MPU6050::ReadAng() {
  int readouts[7];
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

  //获取角度
  fPitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * fRad2Deg;
  fRoll = atan2(2 * q0 * q1 + 2 * q2 * q3, -2 * q1 * q1 - 2 * q2 * q2 + 1) * fRad2Deg;

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;

  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);

 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;

  //更新本次测的时间
  nLastTime = nCurTime;

  MPUAngData[0]= fNewRoll;
  MPUAngData[1]= fNewPitch;
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void MPU6050::WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void MPU6050::ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, 14, true);
  Wire.endTransmission(true);
  for (long i = 0; i < 7; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//对大量读数进行统计，校准平均偏移量
void MPU6050::Calibration()
{
  float valSums[3] = {0.0f, 0.0f, 0.0f};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[7];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < 3; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < 3; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void MPU6050::Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 2; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f; //unit: g
  }
  pRealVals[2] = (float)(pReadout[2] - calibData[2]) / 16384.0f + 1;
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = pReadout[i] / 131.0f;  //250度/s
  }
}

float MPU6050::GetAngRoll(){
  return MPUAngData[0];
}

float MPU6050::GetAngPitch(){
  return MPUAngData[1];
}