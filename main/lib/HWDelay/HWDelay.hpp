/*
 * HWDelay.h
 *
 *  Created on: Mar 1, 2019
 *      Author: fap
 */

#ifndef MAIN_LIB_HWDELAY_HWDELAY_HPP_
#define MAIN_LIB_HWDELAY_HWDELAY_HPP_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define HWDELAY_DEFAULT_DELAY 1000
#define HWDELAY_TIMEOUT_MILLISEC 10000
#define MAX_TIMERS_NUM 3

class HWDelay
{
  public:
	typedef uint16_t hwdHandler_t;
	struct hwdCBargs {
		int timerInd;
		HWDelay* Hinstance;
	};

  private:
	static HWDelay* inst_;   // The one, single instance
	static int  usedTimers;
	static esp_timer_handle_t oneshot_timer[MAX_TIMERS_NUM];
	static bool isValid[MAX_TIMERS_NUM];
	static unsigned long uDelay[MAX_TIMERS_NUM];
	static esp_timer_create_args_t oneshot_timer_args[MAX_TIMERS_NUM];
	static SemaphoreHandle_t semaphore;

  public:
	static EventGroupHandle_t hwdeEG[MAX_TIMERS_NUM];
	static int HWDELAYSET;

  public:
	static HWDelay* getInstance();
	bool Delay(unsigned long microseconds);

	unsigned long GetDelay(hwdHandler_t aTimer);
	bool isTimerValid(hwdHandler_t aTimer);

  private:
	HWDelay();
	~HWDelay();
	HWDelay& operator=(const HWDelay&);
	hwdHandler_t setUp(unsigned long microseconds);
	void deallocateTimer(hwdHandler_t aTimer);

};




#endif /* MAIN_LIB_HWDELAY_HWDELAY_HPP_ */
