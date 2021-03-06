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

#include "appConfig.h"
#include "WeatherStation.h"

#include "lib/Barometer/Barometer.h"
#include "lib/Blinker/Blinker.h"
#include "lib/counter/counter.h"
//#include "lib/DHThumidity/DHThumidity.h"
#include "lib/DHT22/DHT.hpp"
#include "lib/i2c/i2c.h"
#include "lib/MLX90393/MLX90393.h"
#include "lib/NVS/NVS.h"

#include "station.h"


static const char* TAG = "WeSta";

WeatherStation::WeatherStation() {
	// TODO Auto-generated constructor stub

}

WeatherStation::~WeatherStation() {
	// TODO Auto-generated destructor stub
}

bool WeatherStation::initStation()
{
	mQtt = mQttClient::getInstance();

	isAquire = false;
	tempIsCelsius = TEMPERATURE_IS_CELSIUS;

	theStation.Windgauge = new CounterUp(WINDGAUGE_PULSE_PIN);
	theStation.Windgauge->setEdge(true);
	theStation.Windgauge->setFilter(true, 120);
	theStation.Windgauge->pause();

//	theStation.Termogauge = new DHThumidity();
	theStation.Termogauge = new DHT();
	theStation.Termogauge->Setup(DHT_TYPE22, GPIO_NUM_23);

	theStation.i2c = new I2Cmaster(SENSOR_I2CDEV, PIN_I2C_1_SDA, PIN_I2C_1_SCL);
	if( theStation.i2c->init() != ESP_OK) {
		ESP_LOGE(TAG, "Error to create i2c driver !");
	}

	theStation.Barometergauge = new Barometer(theStation.i2c);
	theStation.Directiongauge = new MLX90393(theStation.i2c);

	return false;
}

void WeatherStation::everyTenMinutesTask(void *cx)
{
	WeatherStation *ws = (WeatherStation *)cx;
	ws->isAquire = true;
	ESP_LOGD(TAG, "Start the 10 minutes Task... %d", ws->isAquire);

	// SetUp the counter foer WindSpeed
	/*
	time_t startWindAquire;
	startWindAquire = time (NULL);
	ws->theStation.Windgauge->start(); // clear the counter
	ws->theStation.Windgauge->resume(); // start counting
*/
	// Read the Humidity & Temperature
	//ws->theStation.Termogauge->PowerOn(GPIO_NUM_23,DHT22);
	//ws->theStation.Termogauge->PowerOn(TEMPHUMIDITY_PIN,DHT22);
	//ws->theStation.Termogauge->readHumidity();

	ws->MeteoValues.Humidity = ws->theStation.Termogauge->getHumidity();
	ws->MeteoValues.Temperature = ws->theStation.Termogauge->getTemperature(ws->tempIsCelsius);
	ws->MeteoValues.DevPoint = ws->theStation.Termogauge->getDevPoint();

	// Read Pressure
	ws->theStation.Barometergauge->calibration();
	ws->theStation.Barometergauge->setBaseQuota(20);//WEATERSTATION_ALTITUDE);
	ws->theStation.Barometergauge->readPressure();
	ws->MeteoValues.Pressure = ws->theStation.Barometergauge->getPressureSL();
/*
	// calculate the Wind direction
	ws->theStation.Directiongauge->init(0x5c);
	ws->theStation.Directiongauge->setGainSel(0x05);
	ws->theStation.Directiongauge->setHallConf(0x0C);
	ws->theStation.Directiongauge->setDigitalFiltering(0x05C);
	ws->theStation.Directiongauge->setOverSampling(0x00);
	ws->theStation.Directiongauge->setResolution(1,1,1);

	MLX90393::txyzRaw magField;
	float angle;
	// We can make more then one measurament ?
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
*/
	// put the time stamp
	time_t rawtime;
	struct tm * timeinfo;
	time (&rawtime);
	timeinfo = localtime (&rawtime);
    strftime (ws->MeteoValues.TimeStamp,60,"%c",timeinfo);

    vTaskDelay(8000 / portTICK_PERIOD_MS);

    ws->pubValues();

    ws->isAquire = false;

    vTaskDelete(NULL);
}

bool WeatherStation::setPubClient(mQttClient *aMqttClient)
{
	mQtt = aMqttClient;
	return true;
}

bool WeatherStation::pubValues()
{
	char buf[512];
	NVS *nvs = NVS::getInstance();
	EventBits_t ConBit = 0;
	uint16_t TimeOut = 20000;

	ESP_LOGD(TAG, "Publication of Meteo values ... %d ", TimeOut);
	while((ConBit & mQttClient::MQTTCONNECTED) != mQttClient::MQTTCONNECTED) {
    	ConBit = xEventGroupWaitBits(mQtt->mqttEG,mQttClient::MQTTCONNECTED, false, true, 250 / portTICK_PERIOD_MS);
    	if(--TimeOut == 0) {
    		ESP_LOGE(TAG, "Time Out to connect the mQtt broker. Abort ! ");
    		return(false);
    	}
    }

    mQtt->Publication("App/Name", APPLICATIONNAME,0);
    mQtt->Publication("App/Version",VERSION,0);

    if( nvs->rStr_Dev("station_name",buf,512) != ESP_OK ) strcpy(buf,WEATERSTATION_NAME);
    mQtt->Publication("Station/Name",buf,0);
    if( nvs->rStr_Dev("station_long_s",buf,512) != ESP_OK ) strcpy(buf,WEATERSTATION_LONGITUDE_S);
    mQtt->Publication("Station/Longitude",WEATERSTATION_LONGITUDE_S,0);
    if( nvs->rStr_Dev("station_lat_s",buf,512) != ESP_OK ) strcpy(buf,WEATERSTATION_LATITUDE_S);
    mQtt->Publication("Station/Latitude",WEATERSTATION_LATITUDE_S,0);
    if( nvs->rStr_Dev("station_alt_s",buf,512) != ESP_OK ) strcpy(buf,WEATERSTATION_ALTITUDE_S);
    mQtt->Publication("Station/Altitude",WEATERSTATION_ALTITUDE_S,0);

    mQtt->Publication("Meteo/Time",MeteoValues.TimeStamp,0);
    mQtt->Publication("Meteo/Temperature", MeteoValues.Temperature);
    mQtt->Publication("Meteo/Pressure", MeteoValues.Pressure);
    mQtt->Publication("Meteo/Humidity", MeteoValues.Humidity);
    mQtt->Publication("Meteo/DevPoint", MeteoValues.DevPoint);
    mQtt->Publication("Meteo/WindSpeed", MeteoValues.WindSpeed);
    mQtt->Publication("Meteo/WindAngle", MeteoValues.WindDirection);
    mQtt->Publication("Meteo/WindDirection", convertWindDirection(MeteoValues.WindDirection), 0);
    mQtt->Publication("Meteo/Rain", MeteoValues.RainDrop);
	ESP_LOGI(TAG, "Publication Meteo Values, DONE ! ");
    return true;
}

const char *WeatherStation::convertWindDirection(float angle)
{
	for(int i=0;i<15;i++){
		if(angle < 11.25 + i*22.5) return(rose[i]);
	}
	return(rose[0]);
}
