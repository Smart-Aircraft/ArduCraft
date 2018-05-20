#include "Arduino.h"
#include <Wire.h>
#include <math.h>

#ifndef MS5611_h
#define MS5611_h

#define MS5611_ADDRESS                (0x77)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

#define MS5611_RESOLUTION      		  (0x04)
/*
MS5611_ULTRA_HIGH_RES     (0x08)
MS5611_HIGH_RES           (0x06)
MS5611_STANDARD           (0x04)
MS5611_LOW_POWER          (0x02)
MS5611_ULTRA_LOW_POWER    (0x00)
*/

class MS5611
{
    public:

		void begin();
		void init();
		double readTemperature();
		double readPressure();
		double getAltitude(double pressure);

    private:

		double startPressure = 101325;
		uint16_t fc[6];
		uint8_t ct;
		uint8_t uosr;
		int32_t TEMP2;
		int64_t OFF2, SENS2;

	void readPROM();
	uint16_t readRegister16(uint8_t reg);
	uint32_t readRegister24(uint8_t reg);
	uint32_t readRawTemperature();
	uint32_t readRawPressure();
};

#endif
