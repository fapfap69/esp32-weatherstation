/*
 * HWDelay.cpp
 *
 *  Created on: Mar 1, 2019
 *      Author: fap
 */

#include "HWDelay.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>

static const char* TAG = "HWDELAY";


// Define the static Singleton pointer
HWDelay* HWDelay::inst_ = NULL;
esp_timer_handle_t HWDelay::oneshot_timer[MAX_TIMERS_NUM] = { NULL, NULL };
bool HWDelay::isValid[MAX_TIMERS_NUM] = { false, false };
unsigned long HWDelay::uDelay[MAX_TIMERS_NUM] = { 0, 0 };
EventGroupHandle_t HWDelay::hwdeEG[MAX_TIMERS_NUM] = { NULL };
esp_timer_create_args_t HWDelay::oneshot_timer_args[MAX_TIMERS_NUM] = { NULL, NULL, ESP_TIMER_TASK, NULL };
SemaphoreHandle_t HWDelay::semaphore = NULL;

int HWDelay::HWDELAYSET = BIT0;
int HWDelay::usedTimers = 0;

static void oneshot_timer_callback(void* theHinstance);

HWDelay* HWDelay::getInstance() {
   if (inst_ == NULL) {
	   ESP_LOGE(TAG, "Create HWD timer static hinstance !");
       inst_ = new HWDelay();
   }
   return(inst_);
}


HWDelay::HWDelay() {

	hwdCBargs theCBparam;

	vSemaphoreCreateBinary( semaphore );
	if( semaphore == NULL ) {
		ESP_LOGE(TAG, "Unable to create semaphore. Abort!");
		return;
	}

	for(int i=0;i<MAX_TIMERS_NUM; i++) {
		uDelay[i] = HWDELAY_DEFAULT_DELAY;
		hwdeEG[i] = xEventGroupCreate();
		xEventGroupClearBits(hwdeEG[1], HWDELAYSET);

		oneshot_timer_args[i].callback = &oneshot_timer_callback;
		/* argument specified here will be passed to timer callback function */
		theCBparam.timerInd = i;
		theCBparam.Hinstance = this;
		oneshot_timer_args[i].arg = (void*) &theCBparam;
		oneshot_timer_args[i].name = "HWDelay-timer";
		isValid[i] = false;
	}
	return;

}

HWDelay::~HWDelay()
{
	return;
}

HWDelay::hwdHandler_t HWDelay::setUp(unsigned long microseconds) {

	hwdHandler_t theTimerHandler = 0;

	if(usedTimers == MAX_TIMERS_NUM) {
		ESP_LOGE(TAG, "Max number of timers reached, abort !");
		return(0);
	}
	if(microseconds < 20 || microseconds > 1000000 ) {
		ESP_LOGE(TAG, "Invalid delay value = %lud !", microseconds);
		return(0);
	}
	if( xSemaphoreTake( semaphore, ( TickType_t ) 10 ) == pdTRUE ) { // We were able to obtain the semaphore
		for(int i=0;i<MAX_TIMERS_NUM;i++) {
			if(!isValid[i]) {
				theTimerHandler = (hwdHandler_t)i+1;
				uDelay[theTimerHandler-1] = microseconds;
				isValid[theTimerHandler-1] = true;
				xEventGroupClearBits(hwdeEG[theTimerHandler-1], HWDELAYSET);
				usedTimers++;
			}
		}
		if(theTimerHandler == 0)
			ESP_LOGE(TAG, "No space for timer allocation !");

        xSemaphoreGive( semaphore );
		return(theTimerHandler);
    }
	ESP_LOGE(TAG, "Semaphore hang-up, failed to setup !");
	return(0);
}

void HWDelay::deallocateTimer(hwdHandler_t aTimer) {

	if( xSemaphoreTake( semaphore, ( TickType_t ) 20 ) == pdTRUE ) { // We were able to obtain the semaphore
		usedTimers--;
		uDelay[aTimer-1] = 0;
		isValid[aTimer-1] = false;
		if(oneshot_timer[aTimer-1] != NULL)
			esp_timer_delete(oneshot_timer[aTimer-1]);
		oneshot_timer[aTimer-1] = NULL;
		xEventGroupClearBits(hwdeEG[aTimer-1], HWDELAYSET);
		ESP_LOGD(TAG, "Deallocate timer %d !", aTimer);
		xSemaphoreGive( semaphore );
		return;
	}
	ESP_LOGE(TAG, "Semaphore hang-up, failed to deallocate !");
	return;
}

bool HWDelay::Delay(unsigned long microseconds) {

	HWDelay::hwdHandler_t aTimerHandler = setUp(microseconds);
	if(aTimerHandler == 0) {
		return(false);
	}

	int ind = aTimerHandler - 1;
	esp_err_t err;
	if( esp_timer_create(&oneshot_timer_args[ind], &oneshot_timer[ind]) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create Timer = %ud !", aTimerHandler);
		deallocateTimer(aTimerHandler);
		return(false);
	} else {
		ESP_LOGD(TAG, "Timer %ud created ", aTimerHandler);
	}

	EventBits_t uxBits;
	TickType_t xTicksToWait = HWDELAY_TIMEOUT_MILLISEC / portTICK_PERIOD_MS;

    err = esp_timer_start_once(oneshot_timer[ind], uDelay[ind]);
    if(err != ESP_OK) {
    	ESP_LOGE(TAG, "Failed to start timer = %ud ! (%d)", aTimerHandler, err);
		deallocateTimer(aTimerHandler);
    	return(false);
    }
    ESP_LOGD(TAG, "Started One-shot timer = %ud, time since boot: %lld us", aTimerHandler, esp_timer_get_time());

    uxBits = xEventGroupWaitBits(hwdeEG[ind], HWDELAYSET , pdTRUE, pdTRUE, xTicksToWait);
    if(uxBits != 0) {
    	ESP_LOGE(TAG, "The Delay() for the timer = %ud exits for TIMEOUT, some thing goes wrong ?!", aTimerHandler);
    	deallocateTimer(aTimerHandler);
    	return(false);
    }
    deallocateTimer(aTimerHandler);
    return(true);
}

static void oneshot_timer_callback(void* theTimerParam)
{
    HWDelay::hwdCBargs *par = (HWDelay::hwdCBargs *) theTimerParam;
    HWDelay *theHins = par->Hinstance;
    xEventGroupSetBits(theHins->hwdeEG[par->timerInd], theHins->HWDELAYSET);

    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGD(TAG, "One-shot timer = %d END ! Time since boot: %lld us",par->timerInd+1, time_since_boot);
    return;
}
