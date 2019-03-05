/*
 * Blinker.cpp
 *
 *  Created on: Nov 28, 2018
 *      Author: fap
 */
#include "Blinker.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "rom/gpio.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO

// Define the static Singleton pointer
Blinker* Blinker::inst_ = NULL;
char Blinker::BlinkPattern[MAX_NUM_OF_PATTERN_CHAR+1] =  "OO..OO..OO..OO.." ;
gpio_num_t Blinker::gpioNum = (gpio_num_t)BLINK_GPIO;
TaskHandle_t Blinker::xHandle = NULL;
SemaphoreHandle_t Blinker::semAction = NULL;

Blinker* Blinker::getInstance() {
   if (inst_ == NULL) {
	   printf("Create blinker !\n");
       inst_ = new Blinker();
   }
   return(inst_);
}

Blinker::Blinker()
{
	semAction = xSemaphoreCreateMutex();
    if( semAction == NULL ) {
    	 printf("ERROR create the semaphore!\n");
    } else {
    	gpioNum = (gpio_num_t)BLINK_GPIO;
    	SetPat("OO..OO..OO..OO..");
    	xHandle = NULL;
    }
}
void Blinker::SetPat(const char *blinkPat)
{
	const char *ptr = blinkPat;
	char *dPtr = BlinkPattern;
	if(semAction == NULL) return;

	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		while((ptr - blinkPat) < MAX_NUM_OF_PATTERN_CHAR && *ptr != '\0') {
			*dPtr = *ptr;
			dPtr++;
			ptr++;
		}
		if(strlen(BlinkPattern) == 0) {
			strcpy(BlinkPattern, "OO..OO..OO..OO..");
		}
		xSemaphoreGive( semAction );
	}
	return;
}

void Blinker::StartBlink()
{
	gpio_pad_select_gpio(gpioNum);
    gpio_set_direction(gpioNum, GPIO_MODE_OUTPUT);
    xTaskCreate(&Blinker::blink,"Blink Led" , 2048, NULL, tskIDLE_PRIORITY, &xHandle);
    return;
}

void Blinker::StopBlink()
{
	gpio_pad_select_gpio(gpioNum);
    gpio_set_direction(gpioNum, GPIO_MODE_OUTPUT);
    if(xHandle != NULL) {
    	vTaskDelete(xHandle);
    	xHandle = NULL;
    }
    return;
}

void Blinker::blink( void * pvParameters )
{
	uint8_t ind = 0;
    while(1) {
    	if(xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {

    	switch (BlinkPattern[ind++]) {
    	case 'O':
    	case '1':
    	case 'T':
    		gpio_set_level(gpioNum, 1);
    		break;
    	case '.':
    	case '0':
    	case 'F':
    		gpio_set_level(gpioNum, 0);
    		break;
    	}
    	xSemaphoreGive( semAction );
    	}
        vTaskDelay(PATTERN_TICK_MSEC / portTICK_PERIOD_MS);
        if(ind>=MAX_NUM_OF_PATTERN_CHAR || BlinkPattern[ind] == '\0') ind = 0;
    }
    return;
}

