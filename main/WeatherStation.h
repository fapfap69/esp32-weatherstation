/*
 * WeatherStation.h
 *
 *  Created on: Dec 3, 2018
 *      Author: fap
 */

#ifndef MAIN_WEATHERSTATION_H_
#define MAIN_WEATHERSTATION_H_

#include "appConfig.h"
#include "lib/Barometer/Barometer.h"
#include "lib/counter/counter.h"
#include "lib/DHThumidity/DHThumidity.h"
#include "lib/i2c/i2c.h"
#include "lib/mQttClient/mQttClient.h"
#include "lib/MLX90393/MLX90393.h"


class WeatherStation {

private:
	bool  tempIsCelsius;

	mQttClient *mQtt;

public:
	struct MeteoData {
		float WindSpeed;
		float WindDirection;
		float RainDrop;
		float Humidity;
		float Temperature;
		float DevPoint;
		float Pressure;
		char  TimeStamp[60];
	} MeteoValues;

	struct MeteoStation {
		CounterUp *Windgauge;
		//CounterUp *Raingauge;
		int	Windcounter;
		int RainCounter;
		int RainGlobalCounter;

		I2Cmaster *i2c;

		MLX90393 *Directiongauge;
		DHThumidity *Termogauge;
		Barometer *Barometergauge;

	} theStation;

	bool isAquire;
	const char* rose[16] = {"N","NNE","NE","ENE","E","ESE","SE","SSE",
							"S","SSO","SO","OSO","O","ONO","NO","NNO"};

private:
	const char *convertWindDirection(float angle);


public:
	WeatherStation();
	virtual ~WeatherStation();
	bool initStation();
	static void everyTenMinutesTask(void *theMeteoStation);
	bool setPubClient(mQttClient *aMqttClient);
	bool pubValues();



};

#endif /* MAIN_WEATHERSTATION_H_ */
