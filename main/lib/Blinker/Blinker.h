/*
 * Blinker.h
 *
 *  Created on: Nov 28, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_BLINKER_H_
#define MAIN_LIB_BLINKER_H_

#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "rom/gpio.h"
#include "sdkconfig.h"

#define MAX_NUM_OF_PATTERN_CHAR 16
#define PATTERN_TICK_MSEC 250
#define CONFIG_BLINK_GPIO 5

// ---- BLINK SEQUENCES ------------
#define BKS_ACTIVE_OK	"1100000000000000"
#define BKS_BOOT		"1100000011000000"
#define BKS_REBOOT		"1101010101010000"
#define BKS_ERROR_WIFI	"1111000110000000"
#define BKS_ERROR_MQTT	"1111000110011000"
#define BKS_ERROR_I2C	"1111000111100000"
#define BKS_ERROR_SENS	"1111000101010100"
#define BKS_ERROR_SYS	"1111000110001110"


class Blinker
{
public:
	static Blinker* getInstance();
	void SetPat(const char *blinkPat);
	void SetPin(gpio_num_t gpio_num) { gpioNum=gpio_num; return;};
	void StartBlink();
	void StopBlink();
private:
	static Blinker* inst_;   // The one, single instance
	static SemaphoreHandle_t semAction;
//	static StaticSemaphore_t semBuffer;

	Blinker(); // private constructor
	Blinker(const Blinker&);
	Blinker& operator=(const Blinker&);
	static void blink( void * pvParameters );

protected:
	static char BlinkPattern[MAX_NUM_OF_PATTERN_CHAR+1];
	static gpio_num_t gpioNum;
	static TaskHandle_t xHandle;
};



#endif /* MAIN_LIB_BLINKER_H_ */
