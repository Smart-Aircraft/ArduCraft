#include "PWM.h"
#include "GPS.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "MS5611.h"

#define KP_ROLL           0.2
#define KD_ROLL           0.03

#define KP_PITCH          0.4
#define KD_PITCH          0.03

#define KI_ROLL           0.001
#define KI_PITCH          0.001

#define KP_ROLL_RATE      0.2
#define KI_ROLL_RATE      0.001
#define KD_ROLL_RATE      1.0

#define KP_PITCH_RATE     0.2
#define KI_PITCH_RATE     0.001
#define KD_PITCH_RATE     1.0

#define PIN_RC_MODE       19
#define PIN_RC_ROLL       2
#define PIN_RC_PITCH      3

#define INTERRUPT_MODE    4
#define INTERRUPT_ROLL    0
#define INTERRUPT_PITCH   1
/*
  Pin2  -- interrupt 0
  Pin3  -- interrupt 1
  Pin18 -- interrupt 5
  Pin19 -- interrupt 4
  Pin20 -- interrupt 3
  Pin21 -- interrupt 2
*/

#define PIN_SERVO_ROLL    7
#define PIN_SERVO_PITCH   8

#define MANUAL            0
#define STABLE            1
#define STABLE_TEST       2

int Mode;

double Roll = 0;
double Pitch = 0;

double OutRoll = 0;
double OutPitch = 0;

double RollGoal = 0;
double PitchGoal = 0;

double RollDev = 0;
double PitchDev = 0;

double RollInt = 0;
double PitchInt = 0;

double RollDevLast = 0;
double PitchDevLast = 0;

double RollRate = 0;
double PitchRate = 0;

double RollRateDev = 0;
double PitchRateDev = 0;

double RollRateInt = 0;
double PitchRateInt = 0;
double RollRateDevLast = 0;
double PitchRateDevLast = 0;

volatile long RcRoll = 1500;
volatile long RcRollTimer = 0;
volatile bool RcRollFlag = false;
volatile bool RollChangeFlag = false;

volatile long RcPitch = 1500;
volatile long RcPitchTimer = 0;
volatile bool RcPitchFlag = false;
volatile bool PitchChangeFlag = false;

volatile long RcMode = 1500;
volatile long RcModeTimer = 0;
volatile bool RcModeFlag = false;
volatile bool ModeChangeFlag = false;

volatile int RcRollTemp = 0;
volatile int RcPitchTemp = 0;
volatile int RcModeTemp = 0;
volatile int RcModeTempLast = 0;

unsigned long MainTimer = 0;
unsigned long SafeTimer = 0;

uint8_t SaveSREG;

GPS gps;
MPU6050 Gyro;
HMC5883L Compass;
MS5611 Baro;

void setup() 
{
  Timer4_Initialize();  //设置Pin6 Pin7 Pin8的PWM输出频率为50Hz
  /*
    2  :  TIMER3B
    3  :  TIMER3C
    4  :  TIMER0B
    5  :  TIMER3A
    6  :  TIMER4A
    7  :  TIMER4B
    8  :  TIMER4C
    9  :  TIMER2B
    10 :  TIMER2A
    11 :  TIMER1A
    12 :  TIMER1B
    Timer0 and Timer2 are 8bit timers that can not be used to generate PWM, the rest is 16bit.
    Timer4 --- 6 7 8 used in aircraft
  */
  ServoWrite(PIN_SERVO_ROLL, 0);
  ServoWrite(PIN_SERVO_PITCH, 0);
  //delay(15000);

  Wire.begin(); //初始化Wire库 Wire.begin(400);
  delay(100);
  Gyro.begin(); //初始化陀螺仪
  delay(100);
  
  pinMode(PIN_RC_MODE, INPUT_PULLUP);
  pinMode(PIN_RC_ROLL, INPUT_PULLUP);
  pinMode(PIN_RC_PITCH, INPUT_PULLUP);

  attachInterrupt(INTERRUPT_MODE, GetRcMode, CHANGE);
  attachInterrupt(INTERRUPT_ROLL, GetRcRoll, CHANGE);
  attachInterrupt(INTERRUPT_PITCH, GetRcPitch, CHANGE);
  
  Mode = STABLE;
}

void loop() 
{
  MainTimer = micros();

  Gyro.ReadAng();

  Roll = Gyro.GetAngRoll();
  Pitch = Gyro.GetAngPitch();

  RollRate = Gyro.GetRateRoll();
  PitchRate = Gyro.GetRatePitch();

  if (ModeChangeFlag) {
    ModeChangeFlag = false;
    //清除pid数据
    RollDevLast = 0;
    PitchDevLast = 0;
    RollInt = 0;
    PitchInt = 0;
    RollRateInt = 0;
    PitchRateInt = 0;
    if (RcMode < 1250) {
      Mode = MANUAL;
    } else if (RcMode > 1750) {
      Mode = STABLE_TEST;
    } else {
      Mode = STABLE;
    }
  }
  
  if (RollChangeFlag) {
    RollChangeFlag = false;
    SaveSREG = SREG;
    cli();  //关闭中断
    RollGoal = double(RcRoll - 1500) * 0.125; //如果RcRoll定义成unsigned long，需要先强制转换成long算出来的值才对
    SREG = SaveSREG;  //使能中断
    if (RollGoal > 60.0)
      RollGoal = 60.0;
    if (RollGoal < -60.0)
      RollGoal = -60.0;
  }

  if (PitchChangeFlag) {
    PitchChangeFlag = false;
    SaveSREG = SREG;
    cli();  //关闭中断
    PitchGoal = double(RcPitch - 1500) * 0.125;
    SREG = SaveSREG;  //使能中断
    if (PitchGoal > 60.0)
      PitchGoal = 60.0;
    if (PitchGoal < -60.0)
      PitchGoal = -60.0;
  }

  if (Mode == STABLE) {
    //计算角度误差
    RollDev = 0 - Roll;
    PitchDev = 0 - Pitch;
    //舵机PD控制
    OutRoll = (KP_ROLL * RollDev) - (KD_ROLL * RollRate);
    OutPitch = (KP_PITCH * PitchDev) - (KD_PITCH * PitchRate);

    //自动控制部分限幅
    if (OutRoll > 15.0)
      OutRoll = 15.0;
    if (OutRoll < -15.0)
      OutRoll = -15.0;
    
    if (OutPitch > 15.0)
      OutPitch = 15.0;
    if (OutPitch < -15.0)
      OutPitch = -15.0;

    //手动控制部分
    OutRoll += RollGoal;
    OutPitch += PitchGoal;

  } else if (Mode == STABLE_TEST) {
    //串级pid控制
    //外环PID
    RollDev = 0 - Roll;
    PitchDev = 0 - Pitch;

    RollInt += KI_ROLL * RollDev;
    PitchInt += KI_PITCH * PitchDev;

    if (RollInt > 30.0) RollInt = 30.0;
    if (PitchInt > 30.0) PitchInt = 30.0;
    if (RollInt < -30.0) RollInt = -30.0;
    if (PitchInt < -30.0) PitchInt = -30.0;

    OutRoll = (KP_ROLL * RollDev) + RollInt;
    OutPitch = (KP_PITCH * PitchDev) + PitchInt;

    //内环PID
    //P项
    RollRateDev = OutRoll - (RollDev - RollDevLast);
    PitchRateDev = OutPitch - (PitchDev - PitchDevLast);

    RollDevLast = RollDev;
    PitchDevLast = PitchDev;
    //I项
    RollRateInt += KI_ROLL_RATE * RollRateDev;
    PitchRateInt += KI_PITCH_RATE * PitchRateDev;

    if (RollRateInt > 7.5) RollRateInt = 7.5;
    if (PitchRateInt > 7.5) PitchRateInt = 7.5;
    if (RollRateInt < -7.5) RollRateInt = -7.5;
    if (PitchRateInt < -7.5) PitchRateInt = -7.5;

    //求PID
    OutRoll = KP_ROLL_RATE * RollRateDev + RollRateInt - KD_ROLL_RATE * (RollRateDev - RollRateDevLast);
    OutPitch = KP_PITCH_RATE * PitchRateDev + PitchRateInt - KD_PITCH_RATE * (PitchRateDev - PitchRateDevLast);

    RollRateDevLast = RollRateDev;
    PitchRateDevLast = PitchRateDev;

    //自动控制部分限幅
    if (OutRoll > 15.0)
      OutRoll = 15.0;
    if (OutRoll < -15.0)
      OutRoll = -15.0;
    
    if (OutPitch > 15.0)
      OutPitch = 15.0;
    if (OutPitch < -15.0)
      OutPitch = -15.0;

    //手动控制部分
    OutRoll += RollGoal;
    OutPitch += PitchGoal;

  } else {
    OutRoll = RollGoal;
    OutPitch = PitchGoal;
  }

  OutRoll = -OutRoll;  //试验机舵机特性

  ServoWrite(PIN_SERVO_ROLL, OutRoll);
  ServoWrite(PIN_SERVO_PITCH, OutPitch);

  SafeTimer = micros();
  //使主循环频率为100Hz
  while ((micros() - MainTimer) < 10000) {
    //防止程序卡死
    if ((micros() - SafeTimer) >= 10000) break;
  }

}

void GetRcRoll(){
  RcRollTemp = micros() - RcRollTimer;
  if(digitalRead(PIN_RC_ROLL) == HIGH) {
    RcRollFlag = true;
    RcRollTimer = micros();
  } else {
    if (RcRollFlag) {
      //RcRollTemp = micros() - RcRollTimer;
      RcRollFlag = false;
      if (RcRollTemp - RcRoll > 8 || RcRoll - RcRollTemp > 8) {
        RcRoll = RcRollTemp;
        RollChangeFlag = true;
      }
    }
  }
}

void GetRcPitch(){
  RcPitchTemp = micros() - RcPitchTimer;
  if(digitalRead(PIN_RC_PITCH) == HIGH) {
    RcPitchFlag = true;
    RcPitchTimer = micros();
  } else {
    if (RcPitchFlag) {
      //RcPitchTemp = micros() - RcPitchTimer;
      RcPitchFlag = false;
      if (RcPitchTemp - RcPitch > 8 || RcPitch - RcPitchTemp > 8) {
        RcPitch = RcPitchTemp;
        PitchChangeFlag = true;
      }
    }
  }
}

void GetRcMode(){
  if(digitalRead(PIN_RC_MODE) == HIGH) {
    RcModeFlag = true;
    RcModeTimer = micros();
  } else {
    if (RcModeFlag) {
      RcModeFlag = false;
      RcModeTempLast = RcModeTemp;
      RcModeTemp = micros() - RcModeTimer;
      if ((RcModeTemp - RcMode > 200 || RcMode - RcModeTemp > 200)
          && (RcModeTemp == RcModeTempLast)) {  // 防止信号突变
        RcMode = RcModeTemp;
        ModeChangeFlag = true;
      }
    }
  }
}

//对应脉冲宽度 1830~0.6ms  中值4915~1.5ms  最大值8000~2.4ms
void ServoWrite(int Pin, double Ang) {
  int outAng;
  if (Ang >= 70)
    outAng = 275;
  else if (Ang <= -70)
    outAng = 100;
  else
    outAng = int(187.5 + (Ang * 1.25));
  analogWrite(Pin, outAng);
}
