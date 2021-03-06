/*
 * UpdateFirmaware.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: fap
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>     /* strtoul */

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
#include "esp_err.h"
#include "bootloader_common.h"
#include "esp_image_format.h"
#include "rom/crc.h"

#include "nvs_flash.h"

#include "sdkconfig.h"

#include "../NVS/NVS.h"
#include "../WiFi/WiFi.h"
#include "../mQttClient/mQttClient.h"
#include "../Blinker/Blinker.h"

#include "UpdateFirmware.h"

static const char *TAG = "UpgradeFirmware";
static char ota_write_data[BUFFSIZE + 1] = { 0 };

EventGroupHandle_t UpdateFirmware::egUpdateFirmware = NULL;
const int UpdateFirmware::UPDFRW_NOW = BIT0;
const int UpdateFirmware::UPDFRW_DONE = BIT1;
const int UpdateFirmware::UPDFRW_ERROR = BIT2;

esp_partition_t *UpdateFirmware::partToUpdate = NULL;
char UpdateFirmware::serverUri[256] = DEF_APPSERVER_URI;
const char *UpdateFirmware::tslCertificate = NULL;
bool UpdateFirmware::updateDone = false;
bool UpdateFirmware::updateStart = false;
unsigned long UpdateFirmware::bytesDownloaded = 0;
esp_http_client_handle_t UpdateFirmware::client = NULL;
esp_ota_handle_t UpdateFirmware::update_handle = 0;
esp_err_t UpdateFirmware::lastErrCode = ESP_OK;

uint8_t UpdateFirmware::sha256loader[HASH_LEN]  = { 0 };
uint8_t UpdateFirmware::sha256application[HASH_LEN]  = { 0 };
esp_partition_t *UpdateFirmware::partLoader = NULL;
esp_partition_t *UpdateFirmware::partApplication = NULL;
esp_partition_t *UpdateFirmware::partRunning = NULL;
esp_partition_t *UpdateFirmware::partBoot = NULL;

Blinker *UpdateFirmware::blkLed = Blinker::getInstance();
//WiFi *UpdateFirmware::wifi = WiFi::getInstance();

UpdateFirmware *UpdateFirmware::inst_ = NULL;   // The one, single instance
SemaphoreHandle_t UpdateFirmware::semAction = NULL;
bool UpdateFirmware::isGoodForAOTA = false;
bool UpdateFirmware::isNeedDownload = false;
bool UpdateFirmware::isGoodForDownload = false;


UpdateFirmware* UpdateFirmware::getInstance() {
   if (inst_ == NULL) {
      inst_ = new UpdateFirmware();
   }
   return(inst_);
}


UpdateFirmware::UpdateFirmware() {
	strcpy(serverUri, DEF_APPSERVER_URI);
	partToUpdate = NULL;
	partRunning = NULL;
	partBoot = NULL;
	partLoader = NULL;
	partApplication = NULL;

	updateStart = false;
	updateDone = false;

	update_handle = 0;
	egUpdateFirmware = xEventGroupCreate();
	xEventGroupClearBits(egUpdateFirmware, UPDFRW_NOW | UPDFRW_ERROR | UPDFRW_DONE);

	if(!checkPartTable()) {
		ESP_LOGE(TAG, "Wrong partition table, unable to Update firmware. Abort!");
		setError(ESP_ERR_OTA_PARTITION_CONFLICT, BKS_ERROR_SYS);
	}

	semAction = xSemaphoreCreateMutex();
    if( semAction == NULL ) {
		ESP_LOGE(TAG, "Error to create semaphore !");
    }

}

void UpdateFirmware::http_cleanup(esp_http_client_handle_t client)
{
	if(client != NULL) {
		esp_http_client_close(client);
		esp_http_client_cleanup(client);
	}
    return;
}

esp_err_t UpdateFirmware::checkSha256(const uint8_t *image_hash, const esp_partition_t *aPart)
{
	esp_err_t err;
	uint8_t thePartitioSha[HASH_LEN] = { 0 };
	err = esp_partition_get_sha256(aPart, thePartitioSha);
	if(err != ESP_OK) {
		return(err);
	}
	print_sha256(thePartitioSha, "SHA256 key for the application partition :");
	for(int i=0;i<HASH_LEN;i++)
		if(image_hash[i] != thePartitioSha[i]) {
			ESP_LOGI(TAG, "The SHA256 code differs !");
			return(-1);
		}
	ESP_LOGI(TAG, "SHA256 checks is OK !");
    return(ESP_OK);

}
void UpdateFirmware::print_sha256 (const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
    return;
}

esp_err_t UpdateFirmware::readSha256fromURI(const char* aUri, const char* aCert, uint8_t *image_hash)
{
	esp_err_t err = ESP_OK;
	char shaBuffer[128];

	if (httpGet(aUri, aCert) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get data from http server: %s", esp_err_to_name(err));
		return(err);
	}

    int fetchResponse = esp_http_client_fetch_headers(client);
    if( fetchResponse == ESP_FAIL) {
    	ESP_LOGE(TAG, "Failed to get data from http server: %s", esp_err_to_name(err));
		return(err);
    }
    else if (fetchResponse == 0) {
    	ESP_LOGE(TAG, "Get data from http server returns 0 bytes !");
		return(err);
    }
	ESP_LOGW(TAG, "Start the download of SHA256 file...");
	int data_read = 0;
	int totalData = 0;
    while (1) {
        data_read = esp_http_client_read(client, &shaBuffer[totalData], 128-totalData);
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
           	setError(ESP_ERR_HTTP_INVALID_TRANSPORT, BKS_ERROR_WIFI);
            return(err);
        } else if (data_read > 0) {
        	totalData += data_read;
        	if(totalData > 64) {
                ESP_LOGE(TAG, "Error: Wrong SHA256 File contents !");
               	setError(ESP_ERR_HTTP_INVALID_TRANSPORT, BKS_ERROR_WIFI);
                return(ESP_ERR_HTTP_INVALID_TRANSPORT);
        	}
        } else if (data_read == 0) {
            ESP_LOGI(TAG, "Connection closed,all data received");
            break;
        }
        vTaskDelay(10); // just to feed the watchdog
    }
	blkLed->SetPat(BKS_ACTIVE_OK);

	shaBuffer[totalData] = '\0';
	ESP_LOGD(TAG, "Received data (%d) >>%s<<", totalData, shaBuffer);
	for(int i=(totalData - 1) / 2;i>=0;i--) {
		image_hash[i] = (uint8_t) strtoul((const char*)&(shaBuffer[i*2]), NULL, 16);
		shaBuffer[i*2] = '\0';
	}
	return (ESP_OK);
}

esp_err_t UpdateFirmware::checkCrc32(const uint32_t image_crc, const esp_partition_t *aPart, size_t imageSize)
{
	esp_err_t err;
	uint32_t sliceSize = 4096;

	uint8_t *buffer = (uint8_t *)malloc(sliceSize);
	if(buffer == NULL) {
		ESP_LOGE(TAG, "Error to allocate memory. Abort!");
		return(-1);
	}
	uint32_t readData = 0;
	uint32_t partCrc32 = 0;

	while(readData < imageSize) {
		if(imageSize - readData < sliceSize) sliceSize = imageSize - readData;
		err = esp_partition_read(aPart, readData, buffer, sliceSize);
		if(err != ESP_OK) {
			ESP_LOGE(TAG, "Error to read partition. Abort!");
			free(buffer);
			return(-1);
		}
		partCrc32 = crc32_le(partCrc32, buffer, sliceSize);
		readData += sliceSize;
	}
	ESP_LOGI(TAG, "The partition CRC32 value is = %08X ", partCrc32);

	if(image_crc != partCrc32) {
		ESP_LOGI(TAG, "The CRC32 codes differs !");
		free(buffer);
		return(-1);
	}
	ESP_LOGI(TAG, "CRC32 checks is OK !");
	free(buffer);
    return(ESP_OK);
}

esp_err_t UpdateFirmware::readCrc32fromURI(const char* aUri, const char* aCert, uint32_t *crc32)
{
	esp_err_t err = ESP_OK;
	char crcBuffer[16];

	if (httpGet(aUri, aCert) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get data from http server: %s", esp_err_to_name(err));
		return(err);
	}

    int fetchResponse = esp_http_client_fetch_headers(client);
    if( fetchResponse == ESP_FAIL) {
    	ESP_LOGE(TAG, "Failed to get data from http server: %s", esp_err_to_name(err));
		return(err);
    }
    else if (fetchResponse == 0) {
    	ESP_LOGE(TAG, "Get data from http server returns 0 bytes !");
		return(err);
    }
	ESP_LOGW(TAG, "Start the download of CRC32 file...");
	int data_read = 0;
	int totalData = 0;
    while (1) {
        data_read = esp_http_client_read(client, &crcBuffer[totalData], 16-totalData);
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
           	setError(ESP_ERR_HTTP_INVALID_TRANSPORT, BKS_ERROR_WIFI);
            return(err);
        } else if (data_read > 0) {
        	totalData += data_read;
        	if(totalData > 8) {
                ESP_LOGE(TAG, "Error: Wrong CRC32 File contents !");
               	setError(ESP_ERR_HTTP_INVALID_TRANSPORT, BKS_ERROR_WIFI);
                return(ESP_ERR_HTTP_INVALID_TRANSPORT);
        	}
        } else if (data_read == 0) {
            ESP_LOGI(TAG, "Connection closed,all data received");
            break;
        }
        vTaskDelay(10); // just to feed the watchdog
    }
	blkLed->SetPat(BKS_ACTIVE_OK);

	crcBuffer[totalData] = '\0';
	ESP_LOGD(TAG, "Received data (%d) >>%s<<", totalData, crcBuffer);
	*crc32 = (uint32_t) strtoul((const char*)&(crcBuffer), NULL, 16);
	return (ESP_OK);


}


bool UpdateFirmware::checkPartTable()
{
	uint8_t sha256part[HASH_LEN] = { 0 };
	esp_err_t err;

	isGoodForAOTA = false;
	partLoader = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, LOADER_PART_NAME);
	if(partLoader == NULL) {
		ESP_LOGE(TAG, "AOTA LOADER partition not recognized. Abort !");
		blkLed->SetPat(BKS_ERROR_SYS);
		return(false);
	} else {
		err = esp_partition_get_sha256(partLoader, sha256part);
		switch(err) {
		case ESP_ERR_IMAGE_INVALID:
			ESP_LOGE(TAG, "Loader Partition doesn't have a valid image !");
			return(false);
		case ESP_ERR_INVALID_ARG:
			ESP_LOGW(TAG, "Loader Partition doesn't have SHA256 info !");
			break;
		case ESP_OK:
			print_sha256(sha256part, "SHA-256 for the Loader partition table: ");
			strncpy((char *)sha256loader, (const char*)sha256part, HASH_LEN);
			break;
		default:
			ESP_LOGE(TAG, "Error to get SHA256 info !");
			return(false);
		}
		isGoodForAOTA = true;
	}
	partApplication = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, APPLICATION_PART_NAME);
	if(partApplication == NULL) {
		ESP_LOGE(TAG, "AOTA APPLICATION partition not recognized. Abort !");
		blkLed->SetPat(BKS_ERROR_SYS);
		isGoodForAOTA = false;
		return(false);
	} else {
		isNeedDownload = false;
		err = esp_partition_get_sha256(partApplication, sha256part);
		switch(err) {
		case ESP_ERR_IMAGE_INVALID:
			ESP_LOGW(TAG, "Application Partition doesn't have a valid image !");
			isNeedDownload = true;
			break;
		case ESP_ERR_INVALID_ARG:
			ESP_LOGW(TAG, "Application Partition doesn't have SHA256 info !");
			break;
		case ESP_OK:
			print_sha256(sha256part, "SHA-256 for the Application partition table: ");
			strncpy((char *)sha256application, (const char*)sha256part, HASH_LEN);
			break;
		default:
			ESP_LOGW(TAG, "Error to get SHA256 info !");
			isNeedDownload = true;
			break;
		}
	}

	partRunning = (esp_partition_t*)esp_ota_get_running_partition();
	if(partRunning == NULL) {
		ESP_LOGE(TAG, "AOTA  running partition not recognized. Abort !");
		blkLed->SetPat(BKS_ERROR_SYS);
		isGoodForAOTA = false;
	}
	partBoot = (esp_partition_t*)esp_ota_get_boot_partition();
	if(partBoot == NULL) {
		ESP_LOGE(TAG, "AOTA  boot partition not recognized. Abort !");
		blkLed->SetPat(BKS_ERROR_SYS);
		isGoodForAOTA = false;
	}

	if(partRunning == partLoader) {
		isGoodForDownload = true;
	}
	if(partRunning == partApplication) {
		isGoodForDownload = false;
	}
    return(true);
}

esp_err_t UpdateFirmware::httpGet(const char *aServerUri, const char *aServerCertPem)
{
	esp_err_t err;
	int wifiTimeOut = WIFI_TIMEOUT_CYCLES;
    wifi_ap_record_t ap_info;
    while( esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        vTaskDelay(250 / portTICK_PERIOD_MS);
        if(--wifiTimeOut == 0) {
        	ESP_LOGE(TAG,"Error connecting the Wifi. Abort!");
    		return(setError(ESP_ERR_WIFI_NOT_CONNECT, BKS_ERROR_WIFI));
        }
    }
    ESP_LOGI(TAG, "Connect to Wifi ! Start to Connect to Server:%s",aServerUri);
    if(aServerCertPem == NULL) {
        ESP_LOGE(TAG, "PEM server ertificate ERRROR");
        return(setError(ESP_ERR_WIFI_NOT_CONNECT, BKS_ERROR_WIFI));
    }
    esp_http_client_config_t config;
    memset(&config,0,sizeof(esp_http_client_config_t));
    config.url = aServerUri;
    config.cert_pem = aServerCertPem;
    config.event_handler = UpdateFirmware::_http_event_handler;
    config.buffer_size = BUFFSIZE;
    client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
		return(setError(ESP_ERR_HTTP_CONNECT, BKS_ERROR_WIFI));
    }

    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
		return(setError(ESP_ERR_HTTP_CONNECTING, BKS_ERROR_WIFI));
    }
    return(ESP_OK);
}
esp_err_t UpdateFirmware::Update(const char *aServerUri, const char *aServerCertPem)
{

	strncpy(serverUri, aServerUri, 256);
	tslCertificate = aServerCertPem;

    esp_err_t err = ESP_OK;
    if(!isGoodForAOTA || !isGoodForDownload) {
        ESP_LOGW(TAG, "Reject the AOTA update ! %d %d ",isGoodForAOTA, isGoodForDownload);
        return(setError(ESP_ERR_OTA_PARTITION_CONFLICT, BKS_ERROR_SYS));
    }

	blkLed->SetPat("1100000000000000");
	if (httpGet(aServerUri, aServerCertPem) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get data from http server: %s", esp_err_to_name(err));
		return(setError(ESP_ERR_HTTP_CONNECTING, BKS_ERROR_WIFI));
	}

    int fetchResponse = esp_http_client_fetch_headers(client);
    if( fetchResponse == ESP_FAIL) {
    	ESP_LOGE(TAG, "Failed to get data from http client: %s", esp_err_to_name(err));
		return(setError(ESP_ERR_HTTP_CONNECTING, BKS_ERROR_WIFI));
    }
    else if (fetchResponse == 0) {
    	ESP_LOGE(TAG, "Get data from http client returns 0 bytes !");
		return(setError(ESP_ERR_HTTP_CONNECTING, BKS_ERROR_WIFI));
    }
	ESP_LOGW(TAG, "Start the download ...");
    partToUpdate = partApplication; // update_partition
    xTaskCreate(&UpdateFirmware::download,"Download Firmware" , 8192, NULL, 5, NULL);
    return(ESP_OK);
}

void UpdateFirmware::download(void *ptrPar)
{
	esp_err_t err;
    err = esp_ota_begin((const esp_partition_t*)partToUpdate, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
       	setError(err, BKS_ERROR_SYS);
       	vTaskDelete( NULL );
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    bytesDownloaded = 0;
	xEventGroupSetBits(egUpdateFirmware, UPDFRW_NOW);
	blkLed->SetPat("1111000000000000");
    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
           	setError(ESP_ERR_HTTP_INVALID_TRANSPORT, BKS_ERROR_WIFI);
            vTaskDelete( NULL );
        } else if (data_read > 0) {
        	err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA Write error ! (%s)",  esp_err_to_name(err));
               	setError(ESP_ERR_HTTP_INVALID_TRANSPORT, BKS_ERROR_WIFI);
                vTaskDelete( NULL );
            }
            bytesDownloaded += data_read;
            ESP_LOGD(TAG, "Written image length %lu", bytesDownloaded);
        } else if (data_read == 0) {
            ESP_LOGI(TAG, "Connection closed,all data received");
            break;
        }
        vTaskDelay(10); // just to feed the watchdog
    }
	blkLed->SetPat("1111111100000000");
    ESP_LOGI(TAG, "Total Write binary data length : %lu", bytesDownloaded);
    err = esp_ota_end(update_handle);
    //ESP_LOGI(TAG, "BINGO ");
    if(err != ESP_OK) {
    	ESP_LOGE(TAG, "OTA end failed! %s",esp_err_to_name(err));
    	setError(err, BKS_ERROR_SYS);
        vTaskDelete( NULL );
    }
    update_handle = 0;
	xEventGroupSetBits(egUpdateFirmware, UPDFRW_DONE);
    updateStart = false;
    updateDone = true;
    http_cleanup(client);
   	lastErrCode = ESP_OK;
   	blkLed->SetPat(BKS_ACTIVE_OK);
    vTaskDelete( NULL );
}

esp_err_t UpdateFirmware::CheckUpdate()
{
	esp_err_t err;
	/*
	uint8_t image_hash[HASH_LEN];
	ESP_LOGD(TAG, "Check Update firmware...");
	changeExtension(serverUri, "sha256");
	err = readSha256fromURI(serverUri, tslCertificate, image_hash);
	print_sha256(image_hash, "The SHA256 read from File is=");
	if(err == ESP_OK) {
		err = checkSha256((const uint8_t *)image_hash, partApplication);
		if(err == ESP_OK) {
			return(err);
		}
	}
	*/
	uint32_t theCrc32 = 0;
	ESP_LOGD(TAG, "Check Update firmware...");
	changeExtension(serverUri, "crc32");
	err = readCrc32fromURI(serverUri, tslCertificate, &theCrc32);
	if(err == ESP_OK) {
		err = checkCrc32(theCrc32, partApplication, bytesDownloaded);
	}
	return(err);
}
void UpdateFirmware::changeExtension(char *aFileName, const char *newExtension)
{
	int i = strlen(aFileName);
	int n = strlen(newExtension);
	while(aFileName[i-1] != '.' && i>1) {
		i--;
	}
	if(i>1) {
		for(int j =0;j<=n;j++) aFileName[i++] = newExtension[j];
	}
	return;
}

esp_err_t UpdateFirmware::SwitchPartition()
{
	esp_err_t err;
	if(partToUpdate != NULL && ((partToUpdate == partLoader) || (partToUpdate == partApplication && !isNeedDownload)) ) {
		err = esp_ota_set_boot_partition(partToUpdate);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
			return(setError(err, BKS_ERROR_SYS));
		}
		return(ESP_OK);
	} else {
		ESP_LOGE(TAG, "Undefined partition !");
		return(setError(ESP_ERR_INVALID_ARG, BKS_ERROR_SYS) );
	}
}

void UpdateFirmware::Restart()
{
	ESP_LOGI(TAG, "Prepare to restart system...");
	xTaskCreate(&UpdateFirmware::executeTheReboot, "reboot", 8192, NULL, tskIDLE_PRIORITY, NULL);
	return;
}

void UpdateFirmware::executeTheReboot(void *cx)
{
	blkLed->SetPat(BKS_REBOOT);
	mQttClient 	  *mQtt = mQttClient::getInstance();
	mQtt->Stop();
	http_cleanup(client);
	WiFi  *wifi = WiFi::getInstance();
	wifi->Sleep();
	NVS *nvs = NVS::getInstance();
	nvs->Close();
	ESP_LOGI(TAG, "Reboot now !");
	fflush(stdout);
	blkLed->StopBlink();
	esp_restart();
	vTaskDelete( NULL );
}
esp_err_t UpdateFirmware::SwitchToLoader()
{
	if(isGoodForAOTA) {
		partToUpdate = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
		return(SwitchPartition());
	}
	return(ESP_ERR_OTA_PARTITION_CONFLICT);
}

esp_err_t UpdateFirmware::SwitchToApplication()
{
	if(isGoodForAOTA) {
		partToUpdate = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
		return(SwitchPartition());
	}
	return(ESP_ERR_OTA_PARTITION_CONFLICT);
}
esp_err_t UpdateFirmware::ResetAfterError()
{
	partToUpdate = NULL;
	updateStart = false;
	updateDone = false;

	http_cleanup(client);
	xEventGroupClearBits(egUpdateFirmware, UPDFRW_NOW | UPDFRW_ERROR | UPDFRW_DONE);

	lastErrCode = ESP_OK;

	if(update_handle != 0) {
		lastErrCode = esp_ota_end(update_handle);
		if(lastErrCode != ESP_OK) {
			ESP_LOGE(TAG, "esp_ota_end failed!");
			blkLed->SetPat(BKS_ERROR_SYS);
		}
	}
	return lastErrCode;
}

esp_err_t UpdateFirmware::setError(esp_err_t errNo, const char *blkPat)
{
    http_cleanup(client);
	xEventGroupSetBits(egUpdateFirmware, UPDFRW_ERROR);
	lastErrCode = errNo;
	blkLed->SetPat(blkPat);
	return( errNo );
}

esp_err_t UpdateFirmware::_http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // Write out data
                // printf("%.*s", evt->data_len, (char*)evt->data);
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}


#ifdef LARGE
bool UpdateFirmware::checkPartition(esp_partition_t* otaPartition)
{
	uint8_t sha256boot[HASH_LEN] = { 0 };
	uint8_t sha256run[HASH_LEN] = { 0 };
	uint8_t sha256part[HASH_LEN] = { 0 };
	const esp_partition_t *partFactory;
	const esp_partition_t *partBoot;
	const esp_partition_t *partRunning;

	partRunning = esp_ota_get_running_partition();
	if(esp_partition_get_sha256(partRunning, sha256run) != ESP_OK) {
		ESP_LOGW(TAG, "Running Partition doesn't have SHA256 info !");
	} else {
		print_sha256(sha256run, "SHA-256 for the Running partition table: ");

		partBoot = esp_ota_get_boot_partition();
		if(esp_partition_get_sha256(partBoot, sha256boot) != ESP_OK) {
			ESP_LOGW(TAG, "Boot Partition doesn't have SHA256 info !");
		} else {
			print_sha256(sha256boot, "SHA-256 for the Boot partition table: ");
			if (strcmp((const char*)sha256run,(const char*)sha256boot) != 0) {
				ESP_LOGW(TAG, "Boot Partition and Running Partition are differnt !");
			}
		}
	}
	partFactory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
	if(esp_partition_get_sha256(partFactory, sha256part) != ESP_OK) {
		ESP_LOGW(TAG, "Factory Partition doesn't have SHA256 info !");
	} else {
		print_sha256(sha256part, "SHA-256 for the Factory partition table: ");
	}

	otaPartition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
	if( otaPartition == NULL) {
		ESP_LOGW(TAG, "OTA 0 partition not found. Abort OTA Update !");
		return(false);
	}
	if(esp_partition_get_sha256(otaPartition, sha256part) != ESP_OK) {
		ESP_LOGW(TAG, "OTA 0 Partition doesn't have SHA256 info !");
	}

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    	// OTA app partition table has a smaller NVS partition size than the non-OTA
	    // partition table. This size mismatch may cause NVS initialization to fail.
	    // If this happens, we erase NVS partition and initialize NVS again.
	    if (nvs_flash_erase() != ESP_OK) {
	    	ESP_LOGW(TAG, "Error to erase NVS. Abort OTA Update !");
	    	return(false);
    	}
	    if(nvs_flash_init() != ESP_OK) {
	    	ESP_LOGW(TAG, "Error to init NVS. Abort OTA Update !");
	    	return(false);
	    }
    }
    return(true);
}
#endif
