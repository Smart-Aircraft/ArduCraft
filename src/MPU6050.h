#include "../lib/Kalman/Kalman.h"
#include <Wire.h>
#include <Math.h>
#include "Arduino.h"

#ifndef MPU6050_h
#define MPU6050_h

class MPU6050
{
    public:

        void begin();

        void ReadAng();

        double GetAngRoll();

        double GetAngPitch();

        double GetRateRoll();

        double GetRatePitch();

    private:

        const double halfT = 0.01f;  //需为读取周期的一半
    
        const int MPU = 0x68; //MPU-6050的I2C地址
        const double fRad2Deg = 57.295779f; //将弧度转为角度的乘数
        
        double AngData[2]; //Roll and Pitch
        double RateData[2];

        double fRoll,fPitch;

        double fLastRoll = 0.0f; //上一次滤波得到的Roll角
        double fLastPitch = 0.0f; //上一次滤波得到的Pitch角

        unsigned long nLastTime = 0; //上一次读数的时间

        const int nCalibTimes = 1000; //校准时读数的次数
        int calibData[7]; //校准数据

        volatile double exInt, eyInt, ezInt; // 误差积分
        volatile double q0, q1, q2, q3; // 全局四元数

        double gx,gy,gz;
        double ex,ey,ez;

        const double Ki=0.02f;
        const double Kp=40.0f;

        Kalman kalmanRoll; //Roll角滤波器
        Kalman kalmanPitch; //Pitch角滤波器

    void WriteMPUReg(int nReg, unsigned char nVal);

    void ReadAccGyr(int *pVals);

    void Calibration();

    void Rectify(int *pReadout, double *pRealVals);
};

#endif
