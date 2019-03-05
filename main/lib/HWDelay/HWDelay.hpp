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

#define HWDELAY_TIMEOUT_MILLISEC 10000

class HWDelay
{
  private:
	static HWDelay* inst_;   // The one, single instance
	static esp_timer_handle_t oneshot_timer;
	static bool isValid;
	static unsigned long uDelay;

  public:
	static EventGroupHandle_t hwdeEG;
	static int HWDELAYSET;

  public:
	static HWDelay* getInstance();
	bool Delay();
	bool Delay(unsigned long microseconds);
	bool SetUp(unsigned long microseconds);
	unsigned long GetDelay() { return(uDelay); };
	bool isTimerValid() { return(isValid); };

  private:
	HWDelay(unsigned long microseconds);
	~HWDelay();
	HWDelay& operator=(const HWDelay&);

};




#endif /* MAIN_LIB_HWDELAY_HWDELAY_HPP_ */
