#include "MPU6050.h"

void MPU6050::begin() {
  WriteMPUReg(0x6B, 0x00); //启动MPU6050设备

  //设置角加速度测量范围为-8g ~ 8g
  Wire.beginTransmission(0x68); //开启MPU-6050的传输
  Wire.write(0x1B); //角速度倍率寄存器的地址
  Wire.requestFrom(0x68, 1, true); //先读出原配置
  unsigned char rate_conf = Wire.read();
  rate_conf = ((rate_conf & 0xE7) | (0x02 << 3)); //0x00为2g, 0x03为16g
  Wire.write(rate_conf);
  Wire.endTransmission(true);

  delay(20);

  //设置角速度测量范围为-1000 ~ 1000 deg/s
  Wire.beginTransmission(0x68); //开启MPU-6050的传输
  Wire.write(0x1C); //加速度倍率寄存器的地址
  Wire.requestFrom(0x68, 1, true); //先读出原配置
  unsigned char acc_conf = Wire.read();
  acc_conf = ((acc_conf & 0xE7) | (0x02 << 3));//0x00为250deg/s, 0x03为2000deg/s
  Wire.write(acc_conf);
  Wire.endTransmission(true);

  //设置MPU6050直通模式i2c，因为gy-86的hmc5883的i2c接在mpu6050上
  Wire.beginTransmission(0x68); //开启MPU-6050的传输
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68); //开启MPU-6050的传输
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Calibration(); //执行校准
  q0 = 1.0f , q1 = 0.0f , q2 = 0.0f , q3 = 0.0f;
  exInt = 0.0 ,eyInt = 0.0, ezInt = 0.0;
  kalmanRoll.setAngle(0);
  kalmanPitch.setAngle(0);
  nLastTime = micros(); //记录当前时间
}

void MPU6050::ReadAng() {
  int readouts[7];
  ReadAccGyr(readouts); //读出测量值
  
  double realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  double fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);

  //测量单位化
  for(int i=0;i<3;i++){
    realVals[i]=realVals[i]/fNorm;
  }

  RateData[0] = realVals[4];
  RateData[1] = realVals[5];

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
  double dt = (double)(nCurTime - nLastTime) / 1000000.0;


  //对Roll角和Pitch角进行卡尔曼滤波
  double fNewRoll;
  double fNewPitch;
  
  // 当倾角过大的时候
  if ((fRoll < -90 && AngData[0] > 90) || (fRoll > 90 && AngData[0] < -90)) {
    kalmanRoll.setAngle(fRoll);
    fNewRoll = fRoll;
  } else
    fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  // Invert rate, so it fits the restriced accelerometer reading
  if (abs(fNewRoll) > 90)
    realVals[5] = - realVals[5];
  fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  

  //不处理直接滤波
  //double fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  //double fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);

 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;

  //更新本次测的时间
  nLastTime = nCurTime;

  AngData[0]= fNewRoll;
  AngData[1]= fNewPitch;
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
  double valSums[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[7];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < 7; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < 7; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void MPU6050::Rectify(int *pReadout, double *pRealVals) {
  for (int i = 0; i < 2; ++i) {
    pRealVals[i] = (double)(pReadout[i] - calibData[i]) / 4096.0f; //unit: g
  }
  pRealVals[2] = 1.0f + (double)(pReadout[2] - calibData[2]) / 4096.0f;
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (pReadout[i] - calibData[i]) / 32.768f;  //1000度/s
  }
}

double MPU6050::GetAngRoll(){ return AngData[0]; }
double MPU6050::GetAngPitch(){ return AngData[1]; }
double MPU6050::GetRateRoll(){ return RateData[0]; }
double MPU6050::GetRatePitch(){ return RateData[1]; }
