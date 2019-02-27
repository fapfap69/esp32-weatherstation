/*
 * station.h
 *
 *  Created on: Nov 26, 2018
 *      Author: fap
 */

#ifndef MAIN_STATION_H_
#define MAIN_STATION_H_

#include "driver/gpio.h"

#define WIFI_SSD	"OgiggiaNet"
#define WIFI_PASSWORD	"PaperinO"
#define WIFI_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define WEATERSTATION_NAME "OGIGGIA"
#define WEATERSTATION_TZ "CET+1CEST,M3.5.0/2,M10.5.0/3"
#define WEATERSTATION_ALTITUDE	350
#define WEATERSTATION_LONGITUDE	1.258
#define WEATERSTATION_LATITUDE	41.152
#define WEATERSTATION_ALTITUDE_S	"350 msl"
#define WEATERSTATION_LONGITUDE_S	"1º10' E"
#define WEATERSTATION_LATITUDE_S	"41º15' N"

#define TEMPERATURE_IS_CELSIUS true

#define MQTTCL_BROKER_URL	"mqtt://192.168.178.127:1883"
#define MQTTCL_STATION_NAME	"Test"


#endif /* MAIN_STATION_H_ */
