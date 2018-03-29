#include "MS5611.h"

void MS5611::begin()
{
    Wire.begin();

    Wire.beginTransmission(MS5611_ADDRESS);

    Wire.write(MS5611_CMD_RESET);

    Wire.endTransmission();

    delay(100);

    readPROM();

    delay(100);

    init();
}

void MS5611::init()
{
    double addPressure;
    double realPressure;
    for (int i = 0; i < 150; i ++) {
        realPressure = readPressure();
        if (i >= 50) {
            addPressure += realPressure;
        }
        delay(20);
    }
    startPressure = addPressure / 100;
}

void MS5611::readPROM()
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	    fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t MS5611::readRawTemperature()
{
    Wire.beginTransmission(MS5611_ADDRESS);

	Wire.write(MS5611_CMD_CONV_D2 + MS5611_RESOLUTION);

    Wire.endTransmission();

    delay(3);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t MS5611::readRawPressure()
{
    Wire.beginTransmission(MS5611_ADDRESS);

	Wire.write(MS5611_CMD_CONV_D1 + MS5611_RESOLUTION);

    Wire.endTransmission();

    delay(3);

    return readRegister24(MS5611_CMD_ADC_READ);
}

double MS5611::readPressure()
{
    uint32_t D1 = readRawPressure();

    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    OFF2 = 0;
    SENS2 = 0;

    if (TEMP < 2000)
    {
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
    }

    if (TEMP < -1500)
    {
        OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
        SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
    }

    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    double P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double MS5611::readTemperature()
{
    uint32_t D2 = readRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (TEMP < 2000)
    {
        TEMP2 = (dT * dT) / (2 << 30);
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611::getAltitude(double pressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)startPressure, 0.1902949f)));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg)
{
    uint16_t value;
    Wire.beginTransmission(MS5611_ADDRESS);

    Wire.write(reg);

    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 2);
    while(!Wire.available()) {};

    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();

    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg)
{
    uint32_t value;
    Wire.beginTransmission(MS5611_ADDRESS);

    Wire.write(reg);

    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(!Wire.available()) {};

    uint8_t vxa = Wire.read();
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();

    Wire.endTransmission();

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}