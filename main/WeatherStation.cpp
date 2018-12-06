/*
 * WeatherStation.cpp
 *
 *  Created on: Dec 3, 2018
 *      Author: fap
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_bt.h"

#include "sdkconfig.h"

#include "lib/Blinker.h"
#include "lib/NVS.h"
//#include "lib/WiFi.h"
#include "lib/counter.h"
#include "lib/DHThumidity.h"
#include "lib/Barometer.h"
#include "lib/i2c.h"
#include "lib/MLX90393.h"

#include "appConfig.h"
#include "WeatherStation.h"

WeatherStation::WeatherStation() {
	// TODO Auto-generated constructor stub

}

WeatherStation::~WeatherStation() {
	// TODO Auto-generated destructor stub
}

bool WeatherStation::initStation()
{
	tempIsCelsius = TEMPERATURE_IS_CELSIUS;

	theStation.Windgauge = new CounterUp(PIN_DIG_2);
	theStation.Windgauge->setEdge(true);
	theStation.Windgauge->setFilter(true, 3000);
	theStation.Windgauge->pause();

	theStation.Termogauge = new DHThumidity();

	theStation.i2c = new I2Cmaster(I2C_NUM_1, PIN_I2C_1_SDA, PIN_I2C_1_SCL);
	theStation.Barometergauge = new Barometer(theStation.i2c);

	theStation.Directiongauge = new MLX90393(theStation.i2c);

	return false;
}

void WeatherStation::everyTenMinutesTask(void *cx)
{
	WeatherStation *ws = (WeatherStation *)cx;

	// SetUp the counter foer WindSpeed
	time_t startWindAquire;
	startWindAquire = time (NULL);
	ws->theStation.Windgauge->start(); // clear the counter
	ws->theStation.Windgauge->resume(); // start counting

	// Read the Humidity & Temperature
	ws->theStation.Termogauge->PowerOn(PIN_DIG_3,DHT22);
	ws->theStation.Termogauge->readHumidity();

	ws->MeteoValues.Humidity = ws->theStation.Termogauge->getHumidity();
	ws->MeteoValues.Temperature = ws->theStation.Termogauge->getTemperature(ws->tempIsCelsius);
	ws->MeteoValues.DevPoint = ws->theStation.Termogauge->getDevPoint();

	// Read Pressure
	ws->theStation.Barometergauge->calibration();
	ws->theStation.Barometergauge->setBaseQuota(WEATERSTATION_ALTITUDE);
	ws->theStation.Barometergauge->readPressure();
	ws->MeteoValues.Pressure = ws->theStation.Barometergauge->getPressureSL();

	// calculate the Wind direction
	ws->theStation.Directiongauge->init(0x5c);
	ws->theStation.Directiongauge->setGainSel(0x05);
	ws->theStation.Directiongauge->setHallConf(0x0C);
	ws->theStation.Directiongauge->setDigitalFiltering(0x05C);
	ws->theStation.Directiongauge->setOverSampling(0x00);
	ws->theStation.Directiongauge->setResolution(1,1,1);

	MLX90393::txyzRaw magField;
	float angle;
	ws->theStation.Directiongauge->readRawData(magField);
	angle = atan2(magField.y,magField.x) * 180.0 / M_PI;
	if(angle < 0) angle += 360.0;
	ws->MeteoValues.WindDirection = angle;

	// calculate the Rain drop
	ws->MeteoValues.RainDrop = ws->theStation.RainCounter * 1.0;
	ws->theStation.RainCounter = 0;

	// calculate the Wind speed
	time_t endWindAquire = startWindAquire;
	while(endWindAquire - startWindAquire < 2) { // wait for almost 2 seconds ...
		endWindAquire = time (NULL);
		vTaskDelay( 200 / portTICK_PERIOD_MS );
	}
	ws->theStation.Windcounter = ws->theStation.Windgauge->getValue();
	ws->theStation.Windgauge->pause();
	float period = endWindAquire - startWindAquire;
	ws->MeteoValues.WindSpeed = ws->theStation.Windcounter * 1.0 / period;

	// put the time stamp
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
    strftime (ws->MeteoValues.TimeStamp,60,"%c",timeinfo);

    ws->pubValues();
	return;
}

bool WeatherStation::setPubClient(mQttClient *aMqttClient)
{
	mQtt = aMqttClient;
	return true;
}

bool WeatherStation::pubValues()
{
    mQtt->Publication("/App/Name", APPLICATIONNAME,0);
    mQtt->Publication("/App/Version",VERSION,0);
    mQtt->Publication("/Station/Name",WEATERSTATION_NAME,0);
    mQtt->Publication("/Station/Altitude",WEATERSTATION_ALTITUDE_S,0);
    mQtt->Publication("/Station/Latitude",WEATERSTATION_LATITUDE_S,0);
    mQtt->Publication("/Station/Longitude",WEATERSTATION_LONGITUDE_S,0);
    mQtt->Publication("/Meteo/Time",MeteoValues.TimeStamp,0);
    mQtt->Publication("/Meteo/Temperature", MeteoValues.Temperature);
    mQtt->Publication("/Meteo/Pressure", MeteoValues.Pressure);
    mQtt->Publication("/Meteo/Humidity", MeteoValues.Humidity);
    mQtt->Publication("/Meteo/DevPoint", MeteoValues.DevPoint);
    mQtt->Publication("/Meteo/WindSpeed", MeteoValues.WindSpeed);
    mQtt->Publication("/Meteo/WindAngle", MeteoValues.WindDirection);
    mQtt->Publication("/Meteo/WindDirection", convertWindDirection(MeteoValues.WindDirection), 0);
    mQtt->Publication("/Meteo/Rain", MeteoValues.RainDrop);
    return true;
}

const char *WeatherStation::convertWindDirection(float angle)
{
	for(int i=0;i<15;i++){
		if(angle < 11.25 + i*22.5) return(rose[i]);
	}
	return(rose[0]);
}
