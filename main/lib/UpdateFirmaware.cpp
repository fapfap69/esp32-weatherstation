/*
 * UpdateFirmaware.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: fap
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
#include "esp_err.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "UpdateFirmware.h"

static const char *TAG = "UpgradeFirmware";
static char ota_write_data[BUFFSIZE + 1] = { 0 };

EventGroupHandle_t UpdateFirmware::updateFirmware_EG = NULL;
const int UpdateFirmware::UPDFRW_NOW = BIT0;
const int UpdateFirmware::UPDFRW_DONE = BIT1;
const int UpdateFirmware::UPDFRW_ERROR = BIT2;

esp_partition_t *UpdateFirmware::partToUpdate = NULL;
char UpdateFirmware::serverUri[256] = DEF_APPSERVER_URI;
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
WiFi *UpdateFirmware::wifi = WiFi::getInstance();

UpdateFirmware *UpdateFirmware::inst_ = NULL;   // The one, single instance
SemaphoreHandle_t UpdateFirmware::semAction = NULL;


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
	updateFirmware_EG = xEventGroupCreate();
	xEventGroupClearBits(updateFirmware_EG, UPDFRW_NOW | UPDFRW_ERROR | UPDFRW_DONE);

	semAction = xSemaphoreCreateBinary();
    if( semAction == NULL ) {
    	// ----
    }

}

void UpdateFirmware::http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return;
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

bool UpdateFirmware::checkPartTable()
{
	uint8_t sha256part[HASH_LEN] = { 0 };
	esp_partition_t *partPointer;

	partPointer = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, LOADER_PART_NAME);
	if(partPointer != NULL) {
		if(esp_partition_get_sha256(partPointer, sha256part) != ESP_OK) {
			ESP_LOGW(TAG, "Loader Partition doesn't have SHA256 info !");
		} else {
			print_sha256(sha256part, "SHA-256 for the Loader partition table: ");
			strncpy((char *)sha256loader, (const char*)sha256part, HASH_LEN);
		}
		partLoader = partPointer;
	} else {
		ESP_LOGE(TAG, "AOTA LOADER partition not recognized. Abort !");
		blkLed->SetPat("1000100010001000");
		return(false);
	}
	partPointer = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, APPLICATION_PART_NAME);
	if(partPointer != NULL) {
		if(esp_partition_get_sha256(partPointer, sha256part) != ESP_OK) {
			ESP_LOGW(TAG, "Application Partition doesn't have SHA256 info !");
		} else {
			print_sha256(sha256part, "SHA-256 for the Application partition table: ");
			strncpy((char *)sha256application, (const char*)sha256part, HASH_LEN);
		}
		partApplication = partPointer;
	} else {
		ESP_LOGE(TAG, "AOTA APPLICATION partition not recognized. Abort !");
		blkLed->SetPat("1000100010001000");
		return(false);
	}
	partPointer = (esp_partition_t*)esp_ota_get_running_partition();
	if(partPointer == NULL) {
		ESP_LOGE(TAG, "AOTA  running partition not recognized. Abort !");
		blkLed->SetPat("1000100010001000");
		return(false);
	} else {
		partRunning = partPointer;
	}
	partPointer = (esp_partition_t*)esp_ota_get_boot_partition();
	if(partPointer == NULL) {
		ESP_LOGE(TAG, "AOTA  boot partition not recognized. Abort !");
		blkLed->SetPat("1000100010001000");
		return(false);
	} else {
		partBoot = partPointer;
	}

    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    	// OTA app partition table has a smaller NVS partition size than the non-OTA
	    // partition table. This size mismatch may cause NVS initialization to fail.
	    // If this happens, we erase NVS partition and initialize NVS again.
	    if (nvs_flash_erase() != ESP_OK) {
	    	ESP_LOGW(TAG, "Error to erase NVS. Abort OTA Update !");
			blkLed->SetPat("1000100010001000");
	    	return(false);
    	}
	    if(nvs_flash_init() != ESP_OK) {
	    	ESP_LOGW(TAG, "Error to init NVS. Abort OTA Update !");
			blkLed->SetPat("1000100010001000");
	    	return(false);
	    }
    }
    return(true);
}

esp_err_t UpdateFirmware::Update(const char *aServerUri, const char *aServerCertPem)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_partition_t *update_partition = NULL;
	blkLed->SetPat("1100000000000000");

	if(!checkPartTable()) {
		ESP_LOGE(TAG, "Wrong partition table, unable to Update firmware. Abort!");
		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
		return(ESP_ERR_OTA_PARTITION_CONFLICT);
	}


    ESP_LOGD(TAG, "Starting Application Update... ");
    updateStart = true;

    int wifiTimeOut = WIFI_TIMEOUT_CYCLES;
    wifi_ap_record_t ap_info;
    while( esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK) {
        vTaskDelay(250 / portTICK_PERIOD_MS);
        if(--wifiTimeOut == 0) {
        	ESP_LOGE(TAG,"Error connecting the Wifi. Abort!");
        	return(ESP_ERR_WIFI_NOT_CONNECT);
        }
    }
    ESP_LOGI(TAG, "Connect to Wifi ! Start to Connect to Server....");

    esp_http_client_config_t config;
    strcpy( (char *)config.url , aServerUri);
    strcpy( (char *)config.cert_pem, aServerCertPem);

    client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
        return(ESP_ERR_HTTP_CONNECT);
    }
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
        return(ESP_ERR_HTTP_CONNECTING);
    }
    esp_http_client_fetch_headers(client);

    update_partition = (esp_partition_t*)esp_ota_get_next_update_partition(NULL);
    if( update_partition == NULL) {
        ESP_LOGE(TAG, "Failed to get a valide OTA partition . Abort !");
        http_cleanup(client);
		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
        return(ESP_ERR_OTA_PARTITION_CONFLICT);
    }
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x", update_partition->subtype, update_partition->address);

    err = esp_ota_begin((const esp_partition_t*)update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        http_cleanup(client);
		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
        return(ESP_ERR_OTA_SELECT_INFO_INVALID);
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    xTaskCreate(&UpdateFirmware::download,"Download Firmware" , 8192, NULL, 5, NULL);

    partToUpdate = update_partition;
    return(ESP_OK);
}

void UpdateFirmware::download(void *ptrPar)
{
    bytesDownloaded = 0;
	xEventGroupSetBits(updateFirmware_EG, UPDFRW_NOW);
	blkLed->SetPat("1111000000000000");
    /*deal with all receive packet*/
    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
            http_cleanup(client);
    		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
    		lastErrCode = ESP_ERR_HTTP_INVALID_TRANSPORT;
            return;
        } else if (data_read > 0) {
        	lastErrCode = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (lastErrCode != ESP_OK) {
                http_cleanup(client);
                return;
            }
            bytesDownloaded += data_read;
            ESP_LOGD(TAG, "Written image length %lu", bytesDownloaded);
        } else if (data_read == 0) {
            ESP_LOGI(TAG, "Connection closed,all data received");
            break;
        }
    }
	blkLed->SetPat("1111111100000000");
    ESP_LOGI(TAG, "Total Write binary data length : %lu", bytesDownloaded);
    lastErrCode = esp_ota_end(update_handle);
    if(lastErrCode != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        http_cleanup(client);
		xEventGroupSetBits(updateFirmware_EG, UPDFRW_ERROR);
		return;
    }
	xEventGroupSetBits(updateFirmware_EG, UPDFRW_DONE);
    updateStart = false;
    updateDone = true;
    return;
}


esp_err_t UpdateFirmware::SwitchPartition()
{
	esp_err_t err;
	blkLed->SetPat("10101010");
	if(partToUpdate != NULL) {
		err = esp_ota_set_boot_partition(partToUpdate);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
			return(err);
		}
		return(ESP_OK);
	} else {
		ESP_LOGE(TAG, "Undefined partition !");
		return(ESP_ERR_INVALID_ARG);
	}
}

void UpdateFirmware::Restart()
{
	ESP_LOGI(TAG, "Prepare to restart system!");
	blkLed->SetPat("1111000011100010");
	printf("Restarting now.\n");
	fflush(stdout);
	esp_restart();
}

esp_err_t UpdateFirmware::SwitchToLoader()
{
	blkLed->SetPat("10101010");
	partToUpdate = (esp_partition_t*)esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
	return(SwitchPartition());
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
