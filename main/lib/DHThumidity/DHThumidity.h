/*
DHThumidity.h - Header file for the SensorLib.

Author A.Franco
Date 31/01/2013
Ver. 0.1

*/

#ifndef DHTHUMIDITY_DEF
#define DHTHUMIDITY_DEF

#include <stdint.h>

// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21

#define DHT_OK 0
#define DHT_CHECKSUM_ERROR -1
#define DHT_TIMEOUT_ERROR -2

struct Humiditysensor {
	gpio_num_t PinNumber;
	uint8_t Type;

	float TemperatureC;
	float TemperatureF;
	float Humidity;
	float DevPoint;

};

class DHThumidity {

	public:
		DHThumidity();
		void PowerOn(gpio_num_t, int);

		Humiditysensor readHumidity(void);
		void logError(void);

		float getTemperature(bool isCelsius=false);
		float getHumidity(void);
		float getDevPoint(void);
		float getDevPointFast(void);

		float convertCtoF(float);
		float convertCtoK(float);

	private:
		Humiditysensor sensor;
		bool first;
		time_t lastReadTime;

		int errorCode;
		uint8_t dataBuffer[6];
		bool readSensor(void);
		int getSignalLevel( int usTimeOut, bool state);
};

#endif
