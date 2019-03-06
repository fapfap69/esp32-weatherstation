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
#include <esp_log.h>
#include <math.h>

#define BMP085_DEVICE 0x77    // BMP device address
#define OVERSAMPLING 0 // low power consuming

static const char*TAG = "BAROMETER";

// class constructor
Barometer::Barometer(I2Cmaster *Wire) {

	/*  esp_timer_create_args_t _timerConfig;
	_timerConfig.arg = static_cast<void*>(this);
	_timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(_handleTimer);
	_timerConfig.dispatch_method = ESP_TIMER_TASK;
	_timerConfig.name = "esp32DHTTimer";
	//  esp_timer_create(&_timerConfig, &_timer);
*/
	theI2Cport = Wire;
	pressureSlm = 1013.25; // assume the base pressure SLM
	calibParam = Parameters();
	hwdTimer = HWDelay::getInstance();
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
	unsigned long b4, b7;

//	_task = xTaskGetCurrentTaskHandle(  );
//	esp_timer_create(&_timerConfig, &_timer);

	// First get temperature for compensation
	temperature = calculateTemperature();

	// read the uncalibrated pressure
	uncompensatedPress = readPressureUnc();

	ESP_LOGD(TAG, "temperature :%f uncPressure %lu ", temperature, uncompensatedPress);
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
	pressureSlm = pressure / pow(1-0.0000225577 * baseQuota, 5.25588);
	altitude = calculateAltitude(pressure);
	ESP_LOGD(TAG, "temperature :%f Pressure %5.1f PressureSLM %5.1f ", temperature, pressure, pressureSlm);

//	esp_timer_delete(_timer);

	return(pressure);
}

unsigned long Barometer::readPressureUnc()
{
	unsigned char msb, lsb, xlsb;
	unsigned long up = 0;
	esp_err_t err;
	//uint8_t buffer[4];

	err = writeTo(WRITEREGISTER_ADD, (REQUESTPRESSURE + (OVERSAMPLING<<6)) );
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "writing to read pressure !");
		return -1;
	}

	hwdTimer->Delay(1500 + (3<<OVERSAMPLING) * 1000);
//	esp_timer_start_once(_timer, 1500 + (3<<OVERSAMPLING) * 1000);  // timer is in microseconds
//	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for timer notification

	err = readFrom(PRESSUREREGISTER1_ADD, 1, &msb);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "Reading pressure 1/3 !");
		return -1;
	}
	err = readFrom(PRESSUREREGISTER2_ADD, 1, &lsb);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "Reading pressure 2/3 !");
		return -1;
	}
	err = readFrom(PRESSUREREGISTER3_ADD, 1, &xlsb);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "Reading pressure 3/3 !");
		return -1;
	}

	ESP_LOGD(TAG, "Read 3 bytes %02X %02X %02X", msb, lsb, xlsb);

	up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OVERSAMPLING);

	return up;
}

/* after the 18msec timer do the read !
void Barometer::_handleTimer(Barometer* instance) {
  esp_err_t err;
//  rmt_set_pin(instance->_channel, RMT_MODE_RX, static_cast<gpio_num_t>(instance->sensor.PinNumber));  // reset after using pin as output
//  rmt_rx_start(instance->_channel, true);

//  err = gpio_set_direction((gpio_num_t) instance->sensor.PinNumber, GPIO_MODE_INPUT);
//  err = gpio_set_pull_mode((gpio_num_t) instance->sensor.PinNumber, GPIO_PULLUP_ONLY);

  xTaskNotifyGive(instance->_task);
}
*/
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
	ESP_LOGD(TAG, "Calculate Temperature %u -> %f", uTemp, temp);
	return temp;
}

int Barometer::readTemperatureUnc()
{
	int temp;
	esp_err_t err;

	err = writeTo(WRITEREGISTER_ADD,REQUESTTEMPERATURE);
	if(err != ESP_OK) {
		ESP_LOGE(TAG, "writing to read temperature !");
		return -1;
	}
	//vTaskDelay(5 / portTICK_RATE_MS);
	hwdTimer->Delay(4500);

//	esp_timer_start_once(_timer, 4500);  // timer is in microseconds
//	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for timer notification

	// Read two bytes from registers 0xF6 and 0xF7
	temp = readIntFrom(TEMPERATUREREGISTER_ADD);
	ESP_LOGD(TAG, "Read Unc Temperature %d (%04X)", temp, temp);

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
	ESP_LOGD(TAG, "Calibration AC %04X %04X %04X %04X %04X %04X", calibParam.ac1, calibParam.ac2,
			calibParam.ac3, calibParam.ac4, calibParam.ac5, calibParam.ac6);
	ESP_LOGD(TAG, "Calibration B %04X %04X ", calibParam.b1, calibParam.b2);
	ESP_LOGD(TAG, "Calibration M %04X %04X %04X", calibParam.mb, calibParam.mc, calibParam.md);

	ESP_LOGD(TAG, "Calibration AC %d %d %d %d %d %d", calibParam.ac1, calibParam.ac2,
			calibParam.ac3, calibParam.ac4, calibParam.ac5, calibParam.ac6);
	ESP_LOGD(TAG, "Calibration B %d %d ", calibParam.b1, calibParam.b2);
	ESP_LOGD(TAG, "Calibration M %d %d %d", calibParam.mb, calibParam.mc, calibParam.md);

	return;
}

void Barometer::setBaseQuota(int quota) {
	baseQuota = quota;
	return;
}

int Barometer::getBaseQuota() {
	return(baseQuota);
}


// ----  Private functions -----------------------------------------
// Writes val to address register on device
esp_err_t Barometer::writeTo(uint8_t address, uint8_t val) {

	uint8_t buf[3];
	esp_err_t err;
	buf[0] = address;
	buf[1] = val;
	err = theI2Cport->write(BMP085_DEVICE, buf, 2);
	if(err != ESP_OK) {
		return err;
	}
	return err;
}

// Reads num bytes starting from address register on device in to _buff array
esp_err_t Barometer::readFrom(uint8_t address, int num, uint8_t _buff[]) {

	esp_err_t err;
	err = theI2Cport->write(BMP085_DEVICE, &address, 1);
	if(err != ESP_OK) {
		return err;
	}

	//vTaskDelay(7 / portTICK_RATE_MS);

	err = theI2Cport->read(BMP085_DEVICE, _buff, (size_t)num);
	return err;
}

// Reads Integer starting from address register
int Barometer::readIntFrom(uint8_t address) {
	unsigned int value;
	uint8_t buffer[2];
	esp_err_t err;

	err = readFrom(address, 2, buffer);
	if (err != ESP_OK) {
		return -32767;
	}
	value = buffer[0] << 8 | buffer[1];
	return((int) value);
}
