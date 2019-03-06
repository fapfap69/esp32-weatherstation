/*
ADXL345.cpp - File for the SensorLib.

Author A.Franco
Date 3/03/2019
Ver. 0.1


*/
#include "ADXL345.hpp"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <esp_log.h>
#include <math.h>


static const char*TAG = "ACCELER";

#define ADXL345_DEVICE (0x53)    // ADXL345 device address

// Class constructor
ADXL345::ADXL345(I2Cmaster *Wire) {

	theI2Cport = Wire;

	status = ADXL345_OK;
	errorCode = ADXL345_NO_ERROR;

	accelero = Accelerometer();

	// set the default gains
	accelero.Xgain = 0.00376390;
	accelero.Ygain = 0.00376009;
	accelero.Zgain = 0.00349265;

	return;
}

ADXL345::~ADXL345()
{
	return;
}


// -----------------------  Power section ------------
// Switch On the Chip
void ADXL345::powerOn() {
	//Turning on the ADXL345
	writeTo(ADXL345_POWER_CTL, 0);
	writeTo(ADXL345_POWER_CTL, 16);
	writeTo(ADXL345_POWER_CTL, 8);
	return;
}

bool ADXL345::isLowPower(){
	return getRegisterBit(ADXL345_BW_RATE, 4);
}
void ADXL345::setLowPower(bool state) {
	setRegisterBit(ADXL345_BW_RATE, 4, state);
}
double ADXL345::getRate(){
	uint8_t buffer;
	readFrom(ADXL345_BW_RATE, 1, &buffer);
	buffer &= 0b00001111;
	return (pow(2,((int) buffer)-6)) * 6.25;
}
void ADXL345::setRate(double rate){
	uint8_t readbuffer;
	uint8_t writebuffer;
	int v = (int) (rate / 6.25);
	int r = 0;
	while(v >>= 1)	{
		r++;
	}
	if(r <= 9) {
		readFrom(ADXL345_BW_RATE, 1, &readbuffer);
		writebuffer = (uint8_t) (r + 6) | (readbuffer & 0b11110000);
		writeTo(ADXL345_BW_RATE, writebuffer);
	}
	return;
}

// ---------------- Accelerator section --------------
Accelerometer ADXL345::readAccelerometer() {

	uint8_t buffer[6];

	readFrom(ADXL345_DATAX0, 6, buffer);

	// converting two bytes in to one int
	accelero.Xraw = (((int)buffer[1]) << 8) | buffer[0];
	accelero.Yraw = (((int)buffer[3]) << 8) | buffer[2];
	accelero.Zraw = (((int)buffer[5]) << 8) | buffer[4];

	// convert raw value into eng. values
	accelero.Xint = accelero.Xraw * accelero.Xgain;
	accelero.Yint = accelero.Yraw * accelero.Ygain;
	accelero.Zint = accelero.Zraw * accelero.Zgain;

	// calculate roll & pitch
	calculateRollPitch();

	return(accelero);
}

void ADXL345::calculateRollPitch() {

	double x_val, y_val, z_val;
	double x2, y2, z2;

	x_val = (double)(accelero.Xint);
	y_val = (double)(accelero.Yint);
	z_val = (double)(accelero.Zint);

	x2 = (x_val*x_val);
	y2 = (y_val*y_val);
	z2 = (z_val*z_val);

	accelero.pitch = atan2( x_val , sqrt(y2+z2)) * 57.2957795133 - accelero.pitchBase;
	accelero.roll = atan2( y_val , sqrt(x2+z2)) * 57.2957795133 - accelero.rollBase;
	return;

}



void ADXL345::setOffset(){

	// first reset the Offset
	accelero.rollBase = 0.0;
	accelero.pitchBase = 0.0;
	// calculate the assolute
	calculateRollPitch();
	// set the new offset
	accelero.rollBase = accelero.roll;
	accelero.pitchBase = accelero.pitch;
	return;
}

void ADXL345::setAxisOffset(double rollOffset, double pitchOffset)
{
	accelero.rollBase = rollOffset;
	accelero.pitchBase = pitchOffset;
	return;
}

void ADXL345::getAxisOffset(double* rollOffset, double* pitchOffset)
{
	*rollOffset = accelero.rollBase;
	*pitchOffset = accelero.pitchBase;
	return;
}

double ADXL345::getRoll(void) {
	return(accelero.roll);
}

double ADXL345::getPitch(void) {
	return(accelero.pitch);
}

// -------------- Gain section -------------------
void ADXL345::setAxisGains(double Xgain, double Ygain, double Zgain)
{
	accelero.Xgain = Xgain;
	accelero.Ygain = Ygain;
	accelero.Zgain = Zgain;
	return;
}
void ADXL345::getAxisGains(double* Xgain, double* Ygain, double* Zgain)
{
	*Xgain = accelero.Xgain;
	*Ygain = accelero.Ygain;
	*Zgain = accelero.Zgain;
	return;
}
uint8_t ADXL345::getRangeSetting(void) {
	uint8_t buffer;
	readFrom(ADXL345_DATA_FORMAT, 1, &buffer);
	return(buffer & 0b00000011);
}
void ADXL345::setRangeSetting(int val) {
	uint8_t writebuffer;
	uint8_t readbuffer;

	switch (val) {
		case 2:
			writebuffer = 0b00000000;
			break;
		case 4:
			writebuffer = 0b00000001;
			break;
		case 8:
			writebuffer = 0b00000010;
			break;
		case 16:
			writebuffer = 0b00000011;
			break;
		default:
			writebuffer = 0b00000000;
	}
	readFrom(ADXL345_DATA_FORMAT, 1, &readbuffer);
	writebuffer |= (readbuffer & 0b11101100);
	writeTo(ADXL345_DATA_FORMAT, writebuffer);
	return;
}

// ----  Private functions -----------------------------------------
// Writes val to address register on device
esp_err_t ADXL345::writeTo(uint8_t address, uint8_t val) {

	uint8_t buf[3];
	esp_err_t err;
	buf[0] = address;
	buf[1] = val;
	err = theI2Cport->write(ADXL345_DEVICE, buf, 2);
	if(err != ESP_OK) {
		return err;
	}
	return err;
}

// Reads num bytes starting from address register on device in to _buff array
esp_err_t ADXL345::readFrom(uint8_t address, int num, uint8_t _buff[]) {

	esp_err_t err;
	err = theI2Cport->write(ADXL345_DEVICE, &address, 1);
	if(err != ESP_OK) {
		return err;
	}

	err = theI2Cport->read(ADXL345_DEVICE, _buff, (size_t)num);
	if(err != ESP_OK) {
		status = ADXL345_ERROR;
		errorCode = ADXL345_READ_ERROR;
	}
	return(err);
}
// set bit of a register
esp_err_t ADXL345::setRegisterBit(uint8_t regAdress, int bitPos, bool value) {
	uint8_t buffer;
	esp_err_t err;

	err = readFrom(regAdress, 1, &buffer);
	if(err != ESP_OK) {
		return(err);
	}
	if (value) {
		buffer |= (1 << bitPos);
	} else {
		buffer &= ~(1 << bitPos);
	}
	err = writeTo(regAdress, buffer);
	return(err);
}
// get a gegister bit
bool ADXL345::getRegisterBit(uint8_t regAdress, int bitPos) {
	uint8_t buffer;
	readFrom(regAdress, 1, &buffer);
	return ((buffer >> bitPos) & 1);
}


// EOF
