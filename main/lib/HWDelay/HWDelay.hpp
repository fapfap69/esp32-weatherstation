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
	esp_timer_handle_t oneshot_timer;
	bool isValid;
	unsigned long uDelay;
 public:
	EventGroupHandle_t hwdeEG;
	int HWDELAYSET = BIT0;

  public:
	HWDelay(unsigned long microseconds);
	~HWDelay();
	bool Delay();
	bool Delay(unsigned long microseconds);
	bool SetUp(unsigned long microseconds);
	unsigned long GetDelay() { return(uDelay); };
	bool isTimerValid() { return(isValid); };
};




#endif /* MAIN_LIB_HWDELAY_HWDELAY_HPP_ */
