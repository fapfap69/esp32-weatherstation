/*
 * HWDelay.h
 *
 *  Created on: Mar 1, 2019
 *      Author: fap
 */

#ifndef MAIN_LIB_HWDELAY_HWDELAY_H_
#define MAIN_LIB_HWDELAY_HWDELAY_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

static const char* TAG = "HWDELAY";

#define HWDELAY_TIMEOUT_MILLISEC 10000

class HWDelay
{
  private:
	esp_timer_handle_t oneshot_timer;
	bool isValid;
	unsigned long uDelay;
	EventGroupHandle_t hwdeEG;
	const int HWDELAYSET = BIT0;

  public:
	HWDelay(unsigned long microseconds);
	~HWDelay();
	bool Delay();
	bool Delay(unsigned long microseconds);
	bool SetUp(unsigned long microseconds);
	unsigned long GetDelay() { return(uDelay); };
};




#endif /* MAIN_LIB_HWDELAY_HWDELAY_H_ */
