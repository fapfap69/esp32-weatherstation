/*
 * HWDelay.cpp
 *
 *  Created on: Mar 1, 2019
 *      Author: fap
 */

#include "HWDelay.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"

static const char* TAG = "HWDELAY";


// Define the static Singleton pointer
HWDelay* HWDelay::inst_ = NULL;
esp_timer_handle_t HWDelay::oneshot_timer = NULL;
bool HWDelay::isValid = false;
unsigned long HWDelay::uDelay = 0;
EventGroupHandle_t HWDelay::hwdeEG = NULL;
int HWDelay::HWDELAYSET = BIT0;

static void oneshot_timer_callback(void* theHinstance);

HWDelay* HWDelay::getInstance() {
   if (inst_ == NULL) {
	   printf("Create timer !\n");
       inst_ = new HWDelay(1000);
   }
   return(inst_);
}


HWDelay::HWDelay(unsigned long microseconds) {

	uDelay = microseconds;
	hwdeEG = xEventGroupCreate();
	xEventGroupClearBits(hwdeEG, HWDELAYSET);

	esp_timer_create_args_t oneshot_timer_args;
	oneshot_timer_args.callback = &oneshot_timer_callback;
	/* argument specified here will be passed to timer callback function */
	oneshot_timer_args.arg = (void*) this;
	oneshot_timer_args.name = "one-shot";

	isValid = false;
	if( esp_timer_create(&oneshot_timer_args, &oneshot_timer) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create Timer !");
		return;
	} else {
		ESP_LOGD(TAG, "Timer created ");
		if(!SetUp(microseconds)) return;
	}
	isValid = true;

	return;
}

HWDelay::~HWDelay()
{
	if(oneshot_timer != NULL)
		esp_timer_delete(oneshot_timer);
	return;
}

bool HWDelay::SetUp(unsigned long microseconds) {
	if(microseconds < 20 || microseconds > 1000000 ) {
		ESP_LOGE(TAG, "Invalid delay value = %lud !", microseconds);
		return(false);
	}
	uDelay = microseconds;
	return(true);
}

bool HWDelay::Delay() {
	esp_err_t err;
	EventBits_t uxBits;
	TickType_t xTicksToWait = HWDELAY_TIMEOUT_MILLISEC / portTICK_PERIOD_MS;

    err = esp_timer_start_once(oneshot_timer, uDelay);
    if(err != ESP_OK) {
    	ESP_LOGE(TAG, "Failed to start timer ! (%d)", err);
    	return(false);
    }
    xEventGroupClearBits(hwdeEG, HWDELAYSET);
    ESP_LOGD(TAG, "Started One-shot timer, time since boot: %lld us", esp_timer_get_time());
    uxBits = xEventGroupWaitBits(hwdeEG, HWDELAYSET , pdTRUE, pdTRUE, xTicksToWait);
    if(uxBits != 0) {
    	ESP_LOGE(TAG, "The Delay() exits for TIMEOUT, some thing goes wrong ?!");
    	return(false);
    }
    return(true);
}

bool HWDelay::Delay(unsigned long microseconds) {
	if(SetUp(microseconds))
		return(Delay());
	return(false);
}

static void oneshot_timer_callback(void* theHinstance)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGD(TAG, "One-shot timer END ! Time since boot: %lld us", time_since_boot);
    HWDelay *theHins = (HWDelay *) theHinstance;
    xEventGroupSetBits(theHins->hwdeEG, theHins->HWDELAYSET);
    return;
}
