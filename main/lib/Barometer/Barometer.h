/*
Barometer.h - Header file for the SensorLib.

Author A.Franco
Date 31/01/2013
Ver. 0.1

*/

#ifndef BAROMETER_DEF
#define BAROMETER_DEF

//#include <Arduino.h>
#include "../HWDelay/HWDelay.hpp"
#include "../i2c/i2c.h"

// Barometer based on BMP085

#define WRITEREGISTER_ADD 0xF4
#define TEMPERATUREREGISTER_ADD 0xF6
#define PRESSUREREGISTER1_ADD 0xF6
#define PRESSUREREGISTER2_ADD 0xF7
#define PRESSUREREGISTER3_ADD 0xF8


#define REQUESTTEMPERATURE 0x2E
#define REQUESTPRESSURE 0x34

struct Parameters {
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
	long b5;
};


class Barometer
{
  public:
	Barometer(I2Cmaster *Wire);
	double readPressure();
	void calibration();
	void setBaseQuota(int quota);
	int getBaseQuota();

	double getPressure();
	double getPressureSL();
	double getTemperature();
	double getAltitude();

	double pressure;
	double temperature;
	double altitude;
	double pressureSlm;

  protected:
	unsigned long readPressureUnc();
	double calculateTemperature();
	int readTemperatureUnc();
	int readIntFrom(uint8_t address);
	esp_err_t writeTo(uint8_t address, uint8_t val);
	esp_err_t readFrom(uint8_t address, int num, uint8_t _buff[]);
	double calculateAltitude(double pressure);
	I2Cmaster * theI2Cport;

  private:
	Parameters calibParam;
	unsigned long uncompensatedPress;
	int baseQuota;
	HWDelay *hwdTimer;
//	esp_timer_create_args_t _timerConfig;
//	static void _handleTimer(Barometer* instance);
//	esp_timer_handle_t _timer;
//	TaskHandle_t _task;


};

#endif
