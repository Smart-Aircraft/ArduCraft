#include "Kalman.h"
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

        float GetAngRoll();

        float GetAngPitch();

    private:

        const float halfT = 0.01f;  //需为读取周期的一半
    
        const int MPU = 0x68; //MPU-6050的I2C地址
        const float fRad2Deg = 57.295779f; //将弧度转为角度的乘数
        
        float MPUAngData[2]; //Roll and Pitch

        float fRoll,fPitch;

        float fLastRoll = 0.0f; //上一次滤波得到的Roll角
        float fLastPitch = 0.0f; //上一次滤波得到的Pitch角

        unsigned long nLastTime = 0; //上一次读数的时间

        const int nCalibTimes = 1000; //校准时读数的次数
        int calibData[3]; //校准数据

        volatile float exInt, eyInt, ezInt; // 误差积分
        volatile float q0, q1, q2, q3; // 全局四元数

        float gx,gy,gz;
        float ex,ey,ez;

        const float Ki=0.02f;
        const float Kp=40.0f;

        Kalman kalmanRoll; //Roll角滤波器
        Kalman kalmanPitch; //Pitch角滤波器

    void WriteMPUReg(int nReg, unsigned char nVal);

    void ReadAccGyr(int *pVals);

    void Calibration();

    void Rectify(int *pReadout, float *pRealVals);
};

#endif
