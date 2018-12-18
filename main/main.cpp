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

#include "appConfig.h"
#include "lib/Blinker/Blinker.h"
#include "lib/counter/counter.h"
#include "lib/mQttClient/mQttClient.h"
#include "lib/NVS/NVS.h"
#include "lib/sntp/sntp.h"
#include "lib/SSD1306/SSD1306.h"
#include "lib/UpdateFirmware/UpdateFirmware.h"
#include "lib/WiFi/WiFi.h"
#include "WeatherStation.h"

#include "station.h"

static const char*TAG = "ws_main";

extern "C" {
    void app_main();
}

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");


static esp_err_t mQttCommandCBfunction(esp_mqtt_event_handle_t event, void *cx)
{
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
    gpio_num_t gpio_num = RAINGAUGE_PULSE_PIN; //GPIO_NUM_0;
    assert(rtc_gpio_desc[gpio_num].reg && "GPIO used for pulse counting must be an RTC IO");

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
    int	rainGlobalCount;
    char buf[128];
    char buf1[128];
    int rainPeriodCount = 0;
    BaseType_t xRet;
    TaskHandle_t xTenMin = NULL;

    // set up the led blinker
    Blinker	*blkLed = Blinker::getInstance();
    blkLed->SetPin(PIN_BOARD_LED);
    blkLed->SetPat(BKS_BOOT);
    blkLed->StartBlink();

    /*
    SSD1306 oled = SSD1306(OLED_I2CDEV, OLED_I2CSDA, OLED_I2CSCL);
    oled.begin(0, OLED_ADDRESS, true);
    oled.println("Weather Station v1!\n");
    oled.display();
	*/

    // Start the flash NVS data
    NVS *nvs = NVS::getInstance();
    nvs->Init(DATA_PARTITION_NAME, DEVICE_PARTITION_NAME);

    // Decode the type of running : A first run or a wakeup
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_TIMER  && cause != ESP_SLEEP_WAKEUP_ULP) {
    	ESP_LOGI(TAG, "Not ULP wakeup, initializing ULP");
        init_ulp_program();
    } else {
    	ESP_LOGI(TAG, "TIMER wakeup, ...t");
    	rainPeriodCount = update_pulse_count();
    }

    // Save the acquired rain value
    if( nvs->rInt_NVS("RainCounter", &rainGlobalCount) != ESP_OK ) rainGlobalCount = 0;
    rainGlobalCount += rainPeriodCount;
    nvs->wInt_NVS("RainCounter",rainGlobalCount);

    // Setup the Wifi
    WiFi  *wifi = WiFi::getInstance();
    if( nvs->rStr_Dev("wifi_ssd",buf,WIFI_LEN_SSID) != ESP_OK ) strncpy(buf,WIFI_SSD,WIFI_LEN_SSID);
    if( nvs->rStr_Dev("wifi_passwd",buf1,WIFI_LEN_PASSWORD) != ESP_OK ) strncpy(buf1,WIFI_PASSWORD,WIFI_LEN_PASSWORD);
    wifi->connectAP(buf, buf1, WIFI_MODE_STA);

    // Get the good time
    if( nvs->rStr_Dev("station_tz_s",buf,WIFI_LEN_TZ) != ESP_OK ) strncpy(buf,WEATERSTATION_TZ,WIFI_LEN_TZ);
    obtain_time(wifi, buf);

    // SetUo the Firmware update object
    UpdateFirmware *UpFwr = UpdateFirmware::getInstance();

    // Set Up the mQtt
    mQttClient 	  *mQtt = mQttClient::getInstance();
    if( nvs->rStr_Dev("mqtt_broker_uri",buf,MQTTCL_LEN_BROKER) != ESP_OK ) strncpy(buf,MQTTCL_BROKER_URL,MQTTCL_LEN_BROKER);
    mQtt->setBrokerUri("mqtt://192.168.178.26:1883");//buf);
    if( nvs->rStr_Dev("device_name",buf,MQTTCL_LEN_STATION) != ESP_OK ) strncpy(buf,MQTTCL_STATION_NAME,MQTTCL_LEN_STATION);
    mQtt->setDeviceName("Test");//buf);
    mQtt->init();

    // Set Up the Weather Station
    WeatherStation *theStation = new WeatherStation();
    theStation->initStation();
    theStation->theStation.RainCounter = rainPeriodCount;
    theStation->theStation.RainGlobalCounter = rainGlobalCount;

    // Start the Station activity
   	blkLed->SetPat(BKS_ACTIVE_OK);

    // Espose the Command subscription to the mQtt broker
    mQtt->Subscribe("/Command", mQttCommandCBfunction);

    // Run the Ten Minute Task
    xRet = xTaskCreate(theStation->everyTenMinutesTask,"Ten Minutes Task", 8192, (void *)theStation, tskIDLE_PRIORITY, &xTenMin);
    if(xRet != pdPASS ) {
    	blkLed->SetPat(BKS_ERROR_SYS);
    	abort();
    }

    // and wait for the acquire completation
    do {
    	vTaskDelay(5000 / portTICK_PERIOD_MS);
    	ESP_LOGD(TAG,"Waiting for the end of acquisition...%d",theStation->isAquire);
    } while(theStation->isAquire);

    // Now go in deep sleep mode
    ESP_LOGD(TAG, "Going in ULP Sleep ");
    blkLed->SetPat(BKS_REBOOT);

   	vTaskDelay(1000 / portTICK_PERIOD_MS);
  //  if(xTenMin != NULL) vTaskDelete( xTenMin );

    mQtt->Stop();
    wifi->Sleep();
    nvs->Close();
    blkLed->StopBlink();

    const int wakeup_time_sec = 60;
    ESP_LOGD(TAG,"Enabling timer wakeup, %ds", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);
    esp_deep_sleep_start();

    return;
}
