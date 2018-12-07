/*
Barometer.cpp - File for the SensorLib.

Author A.Franco
Date 31/01/2013
Ver. 0.1


*/
#include "Barometer.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <math.h>

#define BMP085_DEVICE 0x77    // BMP device address
#define OVERSAMPLING 0 // low power consuming



// class constructor
Barometer::Barometer(I2Cmaster *Wire) {
	theI2Cport = Wire;
	pressureSlm = 1013.25; // assume the base pressure SLM
	calibParam = Parameters();
	return;
}

// Interface functions
double Barometer::getPressure() {
	return(pressure);
}
double Barometer::getPressureSL() {
	return(pressureSlm);
}
double Barometer::getTemperature() {
	return(temperature);
}
double Barometer::getAltitude() {
	return(altitude);
}


// Read the pressure
double Barometer::readPressure() {

	long x1, x2, x3, b3, b6, p;
	unsigned long b4, b7, uncPress;

	// First get temperature for compensation
	temperature = calculateTemperature();

	// read the uncalibrated pressure
	uncompensatedPress = readPressureUnc();

	// Compensation
	b6 = calibParam.b5 - 4000;
	// Calculate B3
	x1 = (calibParam.b2 * (b6 * b6)>>12)>>11;
	x2 = (calibParam.ac2 * b6)>>11;
	x3 = x1 + x2;
	b3 = (((((long)calibParam.ac1)*4 + x3)<<OVERSAMPLING) + 2)>>2;

	// Calculate B4
	x1 = (calibParam.ac3 * b6)>>13;
	x2 = (calibParam.b1 * ((b6 * b6)>>12))>>16;
	x3 = ((x1 + x2) + 2)>>2;
	b4 = (calibParam.ac4 * (unsigned long)(x3 + 32768))>>15;

 	b7 = ((unsigned long)(uncompensatedPress - b3) * (50000>>OVERSAMPLING));
  	if (b7 < 0x80000000)
		p = (b7<<1)/b4;
	else
		p = (b7/b4)<<1;

	x1 = (p>>8) * (p>>8);
	x1 = (x1 * 3038)>>16;
	x2 = (-7357 * p)>>16;
	p += (x1 + x2 + 3791)>>4;

	pressure = (double)p / 100.0;  // hPas conversion

	altitude = calculateAltitude(pressure);

	return(pressure);
}

unsigned long Barometer::readPressureUnc()
{
	unsigned char msb, lsb, xlsb;
	unsigned long up = 0;

	writeTo(WRITEREGISTER_ADD, (0x34 + (OVERSAMPLING<<6)) );
	//delay(2 + (3<<OVERSAMPLING));
	vTaskDelay((2 + (3<<OVERSAMPLING)) / portTICK_RATE_MS);

	readFrom(0xF6, 1, &msb);
	readFrom(0xF7, 1, &lsb);
	readFrom(0xF8, 1, &xlsb);

	up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OVERSAMPLING);

	return up;
}

// Calculate the altitude
double Barometer::calculateAltitude(double pressure){

  double A = pressure/(pressureSlm);
  double B = 1.0/5.25588;
  double C = pow(A,B);
  C = 1 - C;
  C = C / 0.0000225577;
  return(C);
}

// Read the temperature
double Barometer::calculateTemperature()
{
	long x1, x2;
	unsigned int uTemp;
	uTemp = readTemperatureUnc();

	x1 = (((long)uTemp - (long)calibParam.ac6)*(long)calibParam.ac5) >> 15;
	x2 = ((long)calibParam.mc << 11)/(x1 + calibParam.md);
	calibParam.b5 = x1 + x2;

	double temp = ((calibParam.b5 + 8)>>4);
	temp = temp/10;
	return temp;
}

int Barometer::readTemperatureUnc()
{
	int temp;

	writeTo(WRITEREGISTER_ADD,RequestTemperature);
  	// Read two bytes from registers 0xF6 and 0xF7
	temp = readIntFrom(TEMPERATUREREGISTER_ADD);
	return(temp);
}


void Barometer::calibration()
{
	calibParam.ac1 = readIntFrom(0xAA);
	calibParam.ac2 = readIntFrom(0xAC);
	calibParam.ac3 = readIntFrom(0xAE);
	calibParam.ac4 = readIntFrom(0xB0);
	calibParam.ac5 = readIntFrom(0xB2);
	calibParam.ac6 = readIntFrom(0xB4);
	calibParam.b1 = readIntFrom(0xB6);
	calibParam.b2 = readIntFrom(0xB8);
	calibParam.mb = readIntFrom(0xBA);
	calibParam.mc = readIntFrom(0xBC);
	calibParam.md = readIntFrom(0xBE);
	return;
}

void Barometer::setBaseQuota(int quota) {

	double p = readPressure(); // hPas
	pressureSlm = p / pow(1-0.0000225577 * quota, 5.25588);
	baseQuota = quota;
	return;
}

int Barometer::getBaseQuota() {

	return(baseQuota);
}


// ----  Private functions -----------------------------------------
// Writes val to address register on device
void Barometer::writeTo(uint8_t address, uint8_t val) {
//	theI2Cport->beginTransmission(BMP085_DEVICE);
	theI2Cport->write(BMP085_DEVICE, &address, 1);
	theI2Cport->write(BMP085_DEVICE, &val, 1);
//	theI2Cport->write(address);
//	theI2Cport->write(val);
//	theI2Cport->endTransmission();
	return;
}

// Reads num bytes starting from address register on device in to _buff array
void Barometer::readFrom(uint8_t address, int num, uint8_t _buff[]) {
//	theI2Cport->beginTransmission(BMP085_DEVICE);
	theI2Cport->write(BMP085_DEVICE, &address, 1);
//	theI2Cport->write(address);
//	theI2Cport->endTransmission();

//	delay(6);
	vTaskDelay(6 / portTICK_RATE_MS);

//	theI2Cport->beginTransmission(BMP085_DEVICE);
//	theI2Cport->requestFrom(BMP085_DEVICE, num);
  theI2Cport->read(BMP085_DEVICE, _buff, (size_t)num);
//	int i = 0;
//	while(theI2Cport->available())	{
//		_buff[i] = Wire.read();
//		i++;
//	}
//	if(i != num){
//		// Error
//	}
//	theI2Cport->endTransmission();
//	return;
	return;
}

// Reads Integer starting from address register
int Barometer::readIntFrom(uint8_t address) {
	unsigned int value;
	uint8_t buffer[2];

	readFrom(address, 2, buffer);
	value = buffer[0] << 8 | buffer[1];
	return((int) value);
}
