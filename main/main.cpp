/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>

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

#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "ulp_main.h"

#include "sdkconfig.h"

#include "lib/mQttClient.h"
#include "lib/UpdateFirmware.h"
#include "lib/Blinker.h"
#include "lib/NVS.h"
#include "lib/WiFi.h"
#include "lib/sntp.h"
#include "lib/counter.h"

#include "appConfig.h"
#include "WeatherStation.h"

static const char*TAG = "appLoader";

extern "C" {
    void app_main();
}

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


static esp_err_t mQttCommandCBfunction(esp_mqtt_event_handle_t event, void *cx)
{
	mQttClient *mQtt = mQttClient::getInstance();
	UpdateFirmware *UpFwr = UpdateFirmware::getInstance();

	char Command[256+1];
	char Parameter[256+1];
	char *ptr = event->data;
	int i = 0;

	while(i<event->data_len && i<256) {
		Command[i] = ptr[i];
		if(Command[i] == '?') break;
		i++;
	}
	Command[i++] = '\0';
	int j = 0;
	while(i<event->data_len && j<256) {
		Parameter[j++] = ptr[i++];
	}
	Parameter[j] = '\0';

	if(strcmp(Command, "UPDATE") == 0) {
		if (UpFwr->SwitchToLoader() == ESP_OK) {
			UpFwr->Restart();
		}
		return(ESP_OK);
	}
	return ESP_OK;
}

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_ulp_program()
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = PIN_DIG_4; //GPIO_NUM_0;
    assert(rtc_gpio_desc[gpio_num].reg && "GPIO used for pulse counting must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_next_edge = 0;
    ulp_io_number = rtc_gpio_desc[gpio_num].rtc_num; /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = 10000;

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_hold_en(gpio_num);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    /* Set ULP wake up period to T = 20ms.
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
    ulp_set_wakeup_period(0, 20000);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static uint32_t update_pulse_count()
{

    /* ULP program counts signal edges, convert that to the number of pulses */
    uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
    ESP_LOGD(TAG, "Pulse count from ULP: %5d", pulse_count_from_ulp);

    /* Save the new pulse count to NVS */
    return (pulse_count_from_ulp);
}



void app_main()
{
    char ssid[WIFI_LEN_SSID+1];
    char password[WIFI_LEN_PASSWORD+1];
    char broker[MQTTCL_LEN_BROKER];
    char station[MQTTCL_LEN_STATION];
    int	rainGlobalCount;

    printf("Weather Station v1!\n");

    // Decode the type of running
    int rainPeriodCount = 0;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_TIMER  && cause != ESP_SLEEP_WAKEUP_ULP) {
    	ESP_LOGI(TAG, "Not ULP wakeup, initializing ULP");
        init_ulp_program();
    } else {
    	ESP_LOGI(TAG, "TIMER wakeup, ...t");
    	rainPeriodCount = update_pulse_count();
    }

    // --- Switch Off
    //::esp_bt_controller_disable();

    NVS *nvs = NVS::getInstance();
    nvs->Init();
    if( nvs->rStr_Dev("wifi_ssd",ssid,WIFI_LEN_SSID) != ESP_OK ) strcpy(ssid,WIFI_SSD);
    if( nvs->rStr_Dev("wifi_password",password,WIFI_LEN_PASSWORD) != ESP_OK ) strcpy(password,WIFI_PASSWORD);
    if( nvs->rStr_Dev("mqtt_broker_uri",broker,MQTTCL_LEN_BROKER) != ESP_OK ) strcpy(broker,MQTTCL_BROKER_URL);
    if( nvs->rStr_Dev("device_name",station,MQTTCL_LEN_STATION) != ESP_OK ) strcpy(station,MQTTCL_STATION_NAME);

    if( nvs->rInt_NVS("RainCounter", &rainGlobalCount) != ESP_OK ) rainGlobalCount = 0;
    rainGlobalCount += rainPeriodCount;
    nvs->wInt_NVS("RainCounter",rainGlobalCount);

    Blinker	*blkLed = Blinker::getInstance();
    blkLed->SetPin(PIN_BOARD_LED);
    blkLed->SetPat("11000000");
    blkLed->StartBlink();

    WiFi  *wifi = WiFi::getInstance();
    wifi->setAP(ssid, password);
    wifi->init();
    wifi->connectAP();

    obtain_time(wifi);

    UpdateFirmware *UpFwr = UpdateFirmware::getInstance();

    mQttClient 	  *mQtt = mQttClient::getInstance();
    mQtt->setBrokerUri(broker);
    mQtt->setDeviceName(station);
    mQtt->init();
    mQtt->Subscribe("/Command", mQttCommandCBfunction);

    WeatherStation *theStation = new WeatherStation();
    theStation->initStation();
    theStation->theStation.RainCounter = rainPeriodCount;
    theStation->theStation.RainGlobalCounter = rainGlobalCount;

    BaseType_t xRet;
    TaskHandle_t xTenMin = NULL;

    xRet = xTaskCreate(theStation->everyTenMinutesTask,"Ten Minutes Task", 256, (void *)&theStation, tskIDLE_PRIORITY, &xTenMin);
    if(xRet != pdPASS )
    	abort();

    bool isFinishedTheJob = false;
    while(!isFinishedTheJob) {
    	vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    // -----
    vTaskDelete( xTenMin );

    mQtt->Stop();
    wifi->Sleep();
    nvs->Close();
    blkLed->StopBlink();
    UpFwr->Restart();

    // ----
    ESP_LOGI(TAG, "ULP Sleep ");
    const int wakeup_time_sec = 60;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    //esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();

}
