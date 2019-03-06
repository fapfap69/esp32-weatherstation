/*
 * DHT22.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_DHT22_DHT_HPP_
#define MAIN_LIB_DHT22_DHT_HPP_

#pragma once

extern "C" {
  #include <freertos/FreeRTOS.h>
  #include <freertos/task.h>
  #include <freertos/ringbuf.h>
//  #include <esp32-hal-gpio.h>
  #include <driver/rmt.h>
  #include <esp_timer.h>
//  #include <functional>
}

#include "../HWDelay/HWDelay.hpp"

//namespace esp32DHTInternals {

//typedef std::function<void(float, float)> OnData_CB;
//typedef std::function<void(uint8_t)> OnError_CB;

//}  // end namespace esp32DHTInternals

#define RMT_CLK_DIV 80

#define DHT_OK 1
#define DHT_ERR_NODATA 0
#define DHT_ERR_TIMEOUT 2
#define DHT_ERR_NOTACK 3
#define DHT_ERR_DATA 4
#define DHT_ERR_CHECKSUM 5

#define DHT_TYPE22 1
#define DHT_TYPE11 2


class DHT {
	struct Humiditysensor {
		gpio_num_t PinNumber;
		time_t lastReadTime;
		float TemperatureC;
		float TemperatureF;
		float Humidity;
		float DevPoint;
	};

	protected:
	  uint8_t _status;
	  uint8_t _data[5];

	private:
	  Humiditysensor sensor;
	  rmt_channel_t _channel;
//	  esp_timer_handle_t _timer;
//	  TaskHandle_t _task;
	  uint8_t _tipo;
	  RingbufHandle_t _ringBuf;
	  HWDelay *hwdTimer;
//	  esp_timer_create_args_t _timerConfig;

	public:
		DHT();
		~DHT();
		void Setup(uint8_t type, gpio_num_t pin, rmt_channel_t channel = RMT_CHANNEL_0);  // setPin does complete setup of DHT lib
		const char *getError() const;
		const char *GetType() { if(_tipo == DHT_TYPE11) return("DHT11"); else return("DHT22");}

		float getTemperature(bool isCelsius=false);
		float getHumidity(void);
		float getDevPoint(void);
		float getDevPointFast(void);

		float convertCtoF(float);
		float convertCtoK(float);

	private:
  		float _decodeTemperature();
  		float _decodeHumidity();
		void _readSensor();
//  		static void _handleTimer(DHT* instance);
  		void _decode(rmt_item32_t* data, int numItems);
  		void _updatevalues();

};


#endif /* MAIN_LIB_DHT22_DHT_HPP_ */
