/*
 * sntp.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: fap
 */
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/apps/sntp.h"
#include "esp_log.h"

#include "WiFi.h"

#include "../appConfig.h"

static const char *TAG = "SNTP";

bool initialize_sntp(WiFi *theWifi)
{
	int retry_count = 10;
	while (!theWifi->isConnectedToAP()) {
		if(retry_count-- == 0) return(false);
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, (char *)"pool.ntp.org");
    sntp_init();
    return(true);
}

void obtain_time(WiFi *theWifi)
{
    if(!initialize_sntp(theWifi))
    	return;
    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0,0,0,0,0,0,0,0,0 };
    int retry = 0;
    const int retry_count = 10;
    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", WEATERSTATION_TZ, 1);
    tzset();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    time(&now);
    localtime_r(&now, &timeinfo);
    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Rome is: %s", strftime_buf);
    return;
}

