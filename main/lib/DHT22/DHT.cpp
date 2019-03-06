/*
 * DHT22.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: fap
 */

#include "DHT.hpp"  // NOLINT

#include <time.h>
#include <math.h>
#include <esp_log.h>


static const char*TAG = "DHT";

DHT::DHT() {
	_status = DHT_ERR_NODATA;
	_data[0] =_data[1] =_data[2] =_data[3] =_data[4] =_data[5] = 0;
	_channel = RMT_CHANNEL_0;
	_tipo = 0;
	_ringBuf = nullptr;
	hwdTimer = HWDelay::getInstance();
}


DHT::~DHT() {
  if (_channel) {
    rmt_driver_uninstall(_channel);
  }
}

void DHT::Setup(uint8_t senstype, gpio_num_t pin, rmt_channel_t channel) {

  ESP_LOGD(TAG, "Set up the sensor on the pin  %d", pin);

  sensor.PinNumber = pin;
  sensor.lastReadTime = 0;
  _channel = channel;
  _tipo = senstype;

/* esp_timer_create_args_t _timerConfig;
  _timerConfig.arg = static_cast<void*>(this);
  _timerConfig.callback = reinterpret_cast<esp_timer_cb_t>(_handleTimer);
  _timerConfig.dispatch_method = ESP_TIMER_TASK;
  _timerConfig.name = "esp32DHTTimer";
//  esp_timer_create(&_timerConfig, &_timer);
*/
  rmt_config_t config;
  config.rmt_mode = RMT_MODE_RX;
  config.channel = _channel;
  config.gpio_num = sensor.PinNumber;
  config.mem_block_num = 2;
  config.rx_config.filter_en = 1;
  config.rx_config.filter_ticks_thresh = 10;
  config.rx_config.idle_threshold = 1000;
  config.clk_div = RMT_CLK_DIV;
  rmt_config(&config);
  rmt_driver_install(_channel, 400, 0);  // 400 words for ringbuffer containing pulse trains from DHT
  rmt_get_ringbuf_handle(_channel, &_ringBuf);
  esp_err_t err;
  err = gpio_set_direction(sensor.PinNumber, GPIO_MODE_OUTPUT);
  err = gpio_set_pull_mode(sensor.PinNumber, GPIO_PULLUP_ONLY);
  err = gpio_pullup_en(sensor.PinNumber);
  err = gpio_set_level(sensor.PinNumber, 1 ); //
}


const char* DHT::getError() const {
  if (_status == DHT_ERR_TIMEOUT) {
    return "TimeOut";
  } else if (_status == DHT_ERR_NOTACK) {
    return "NotAcknoledge";
  } else if (_status == DHT_ERR_DATA) {
    return "DataError";
  } else if (_status == DHT_ERR_CHECKSUM) {
    return "CeckSum";
  } else if (_status == DHT_ERR_NODATA) {
	return "NoData";
  }
  return "OK";
}

float DHT::getTemperature(bool isCelsius) {
	_updatevalues();
	if(isCelsius) return(sensor.TemperatureC);
	else return(sensor.TemperatureF);
}

float DHT::getHumidity(void) {
	_updatevalues();
	return(sensor.Humidity);
}

float DHT::getDevPoint(void) {
	_updatevalues();
	float A0, SUM;
	A0 = 373.15/(273.15 + sensor.TemperatureC);
	SUM = -7.90298 * (A0-1);
	SUM += 5.02808 * log10(A0);
	SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
	SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
	SUM += log10(1013.246);
	float VP = pow(10, SUM-3) * sensor.Humidity;
	float T = log(VP/0.61078);   // temp var
	sensor.DevPoint = (241.88 * T) / (17.558-T);
	return (sensor.DevPoint);
}

float DHT::getDevPointFast(void) {
	_updatevalues();
	double a = 17.271;
	double b = 237.7;
	double temp = (a * sensor.TemperatureC) / (b + sensor.TemperatureC) + log(sensor.Humidity/100);
	double Td = (b * temp) / (a - temp);
	return Td;
}

// Convert celsius to Farhenait
float DHT::convertCtoF(float c)
{
	return c * 1.8 + 32.0;
}

//Celsius to Kelvin conversion
float DHT::convertCtoK(float c)
{
	return c + 273.15;
}

// --------------------
void DHT::_updatevalues()
{
	int Attempt = 3;
	time_t acttime;
	time( &acttime );
	ESP_LOGD(TAG, "update value act time=%lu  last read=%lu ",acttime ,sensor.lastReadTime );
	if( acttime > (sensor.lastReadTime + 2)) {
		do {
			ESP_LOGD(TAG, "update value Attempt %d ",Attempt);
			vTaskDelay(4000 / portTICK_PERIOD_MS); // wait 4 sec ... could be reduced
			_readSensor();
		} while (_status != DHT_OK && (--Attempt != 0));
	}
}

void DHT::_readSensor() {
	esp_err_t err;
	size_t rx_size = 0;

	ESP_LOGD(TAG, "Start the reading of sensor...");
//	_task = xTaskGetCurrentTaskHandle(  );
//	esp_timer_create(&_timerConfig, &_timer);
	_data[0] = _data[1] = _data[2] = _data[3] = _data[4] = 0;
	_status = DHT_ERR_NODATA;

	// sensor.PinNumber should be set to OUTPUT and HIGH
	err = gpio_set_direction(sensor.PinNumber, GPIO_MODE_OUTPUT);
	err = gpio_set_level(sensor.PinNumber, 0 ); //

//	esp_timer_start_once(_timer, 18 * 1000);  // timer is in microseconds

//	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for timer notification
	hwdTimer->Delay(18 * 1000);
	rmt_set_pin(_channel, RMT_MODE_RX, static_cast<gpio_num_t>(sensor.PinNumber));  // reset after using pin as output
	rmt_rx_start(_channel, true);
	err = gpio_set_direction((gpio_num_t) sensor.PinNumber, GPIO_MODE_INPUT);
	err = gpio_set_pull_mode((gpio_num_t) sensor.PinNumber, GPIO_PULLUP_ONLY);

    //vTaskDelay(5); // wait for the maximum receive time
    hwdTimer->Delay(5000);
//	esp_timer_start_once(_timer, 5 * 1000);  // timer is in microseconds

//	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // wait for timer notification

	rmt_item32_t* items = static_cast<rmt_item32_t*>(xRingbufferReceive(_ringBuf, &rx_size, 1000));
	ESP_LOGD(TAG, "Now we have data. %d Items",(int)rx_size/sizeof(rmt_item32_t));
	if (items) {
		_decode(items, rx_size/sizeof(rmt_item32_t));
		vRingbufferReturnItem(_ringBuf, static_cast<void*>(items));
	} else {
		_status = DHT_ERR_TIMEOUT;  // timeout error
		ESP_LOGW(TAG, "Sensor TimeOut Error !");
	}
	rmt_rx_stop(_channel);
	err = gpio_set_direction(sensor.PinNumber, GPIO_MODE_OUTPUT);
	gpio_set_level(sensor.PinNumber, 1 ); //
	ESP_LOGD(TAG, "End reading ! (Exitus = %s)", getError());

//	esp_timer_delete(_timer);

	return;
}


/* after the 18msec timer do the read !
void DHT::_handleTimer(DHT* instance) {
  esp_err_t err;
//  rmt_set_pin(instance->_channel, RMT_MODE_RX, static_cast<gpio_num_t>(instance->sensor.PinNumber));  // reset after using pin as output
//  rmt_rx_start(instance->_channel, true);

//  err = gpio_set_direction((gpio_num_t) instance->sensor.PinNumber, GPIO_MODE_INPUT);
//  err = gpio_set_pull_mode((gpio_num_t) instance->sensor.PinNumber, GPIO_PULLUP_ONLY);

  xTaskNotifyGive(instance->_task);
}
*/

/*
 *
 * #include "esp_attr.h"
 * void IRAM_ATTR rmt_isr_handler(void* arg)
 * {
 *    //read RMT interrupt status.
 *    uint32_t intr_st = RMT.int_st.val;
 *
 *    //you will find which channels have triggered an interrupt here,
 *    //then, you can post some event to RTOS queue to process the event.
 *    //later we will add a queue in the driver code.
 *
 *   //clear RMT interrupt status.
 *    RMT.int_clr.val = intr_st;
 * }
 *
 */

void DHT::_decode(rmt_item32_t* data, int numItems) {
  ESP_LOGD(TAG, " Decode data -> %d ",numItems );
  for(int i=0;i<numItems;i++) {
  	  printf("%d(%u,%u)  ", i, data[i].duration0 , data[i].duration1);
  }
  if (numItems < 41) {
    _status = DHT_ERR_NOTACK;
	ESP_LOGW(TAG, "Sensor NoAck. Error to read Items !");
  } else if ((data[0].duration0 + data[0].duration1) < 100 || (data[0].duration0 + data[0].duration1) > 190) {
    _status = DHT_ERR_TIMEOUT;
	ESP_LOGW(TAG, "Sensor TimeOut. Error to read Start Framework !(%d,%d)",data[0].duration0 , data[0].duration1);
  } else {
    for (uint8_t i = 1; i < numItems; ++i) {  // don't include tail
      uint8_t pulse = data[i].duration0 + data[i].duration1;
      if (pulse > 55 && pulse < 145) {
        _data[(i - 1) / 8] <<= 1;  // shift left
        if (pulse > 110) {
          _data[(i - 1) / 8] |= 1;
        }
      } else {
        _status = DHT_ERR_DATA;  // DATA error
    	ESP_LOGW(TAG, "Sensor Data Error. Error to read Data Framework !");
        return;
      }
    }
    ESP_LOGD(TAG, " Aquired data -> 0x%2X 0x%2X 0x%2X 0x%2X  0x%2X ",_data[0] ,_data[1] , _data[2] , _data[3] , _data[4] );
    if (_data[4] == ((_data[0] + _data[1] + _data[2] + _data[3]) & 0xFF)) {
      _status = DHT_OK;
      time(&(sensor.lastReadTime));
      sensor.TemperatureC = _decodeTemperature();
      sensor.TemperatureF = convertCtoF(sensor.TemperatureC);
      sensor.Humidity = _decodeHumidity();
      ESP_LOGD(TAG, "Sensor eng. values T=%4.1fC T=%3.1fF H=%3.1f ",sensor.TemperatureC,sensor.TemperatureF, sensor.Humidity);
    } else {
      _status = DHT_ERR_CHECKSUM;  // checksum error
      ESP_LOGW(TAG, "Sensor CheckSum. Error to Calculate data CheckSum !");
    }
  }
}



float DHT::_decodeTemperature() {
  if (_status != DHT_OK) return -32768;
  if( _tipo == DHT_TYPE11)
	  return static_cast<float>(_data[2]);
  else {
	  float temp = ((((uint32_t)(_data[2] & 0x7F)) << 8) | (uint32_t)_data[3]) * 0.1;
	  if (_data[2] & 0x80) {  // negative temperature
	    temp = -temp;
	  }
	  return temp;
  }
}

float DHT::_decodeHumidity() {
  if (_status != DHT_OK) return -1;
  if( _tipo == DHT_TYPE11)
	  return static_cast<float>(_data[0]);
  else {
	  uint32_t temp = _data[0];
	  return ((temp << 8) | (uint32_t)_data[1]) * 0.1;
  }
}

