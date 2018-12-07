/*
 * NVS.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: fap
 */

#include "NVS.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>



#include "sdkconfig.h"

nvs_handle NVS::generalNVS = 0;
nvs_handle NVS::perdeviceNVS = 0;
const char *NVS::TAG = "NVS";
Blinker *NVS::blkLed = Blinker::getInstance();

NVS* NVS::inst_ = NULL;   // The one, single instance
SemaphoreHandle_t NVS::semAction = NULL;
char NVS::namePartData[64] = "";
char NVS::namePartDevice[64] = "";


NVS* NVS::getInstance() {
   if (inst_ == NULL) {
      inst_ = new NVS();
   }
   return(inst_);
}

NVS::NVS()
{
}

bool NVS::Init(const char* parData, const char* parDevice)
{
	strncpy(namePartData, parData, 64);
	strncpy(namePartDevice, parDevice, 64);
	// Initialize NVS.
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		// OTA app partition table has a smaller NVS partition size than the non-OTA
		// partition table. This size mismatch may cause NVS initialization to fail.
		// If this happens, we erase NVS partition and initialize NVS again.
		if (nvs_flash_erase() != ESP_OK) {
		 	ESP_LOGW(TAG, "Error to erase NVS. Abort OTA Update !");
		 	blkLed->SetPat("11001100");
		  	return(false);
	    }
		if(nvs_flash_init() != ESP_OK) {
		  	ESP_LOGW(TAG, "Error to init NVS. Abort OTA Update !");
		 	blkLed->SetPat("11001100");
		   	return(false);
		}
	}
	err = nvs_open(namePartData, NVS_READWRITE, &generalNVS);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
		generalNVS = 0;
	 	blkLed->SetPat("11001100");
		return(false);
	}
	err = nvs_open(namePartDevice, NVS_READWRITE, &perdeviceNVS);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "Error (%s) opening DEVICE NVS handle!", esp_err_to_name(err));
		perdeviceNVS = 0;
	 	blkLed->SetPat("11001100");
		return(false);
	}
	return(true);
}

void NVS::Close()
{
	if(generalNVS != 0) {
		nvs_close(generalNVS);
	}
	if(perdeviceNVS != 0) {
		nvs_close(perdeviceNVS);
	}
	return;
}

esp_err_t NVS::readNVSString(nvs_handle aHandle, const char *aKey, char *aValue, size_t maxLen)
{
	esp_err_t err = ESP_OK;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		err = nvs_get_str(aHandle, aKey, aValue, &maxLen);
		xSemaphoreGive(semAction);
	}
    switch (err) {
    case ESP_OK:
     	ESP_LOGD(TAG, "Read %s = %s", aKey, aValue);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
      	ESP_LOGW(TAG, "Read %s value not Found!", aKey);
       	strcpy(aValue,"");
        break;
    default:
       	ESP_LOGW(TAG, "Error (%s) reading!!", esp_err_to_name(err));
       	break;
    }
	return(err);
}

esp_err_t NVS::readNVSi32(nvs_handle aHandle, const char *aKey, int32_t *aValue)
{
	esp_err_t err = ESP_OK;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		err = nvs_get_i32(aHandle, aKey, aValue);
		xSemaphoreGive(semAction);
	}
    switch (err) {
    case ESP_OK:
      	ESP_LOGD(TAG, "Read %s = %d", aKey, *aValue);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
      	ESP_LOGW(TAG, "Read %s value not Found!", aKey);
      	*aValue = 0;
        break;
    default:
       	ESP_LOGW(TAG, "Error (%s) reading!!", esp_err_to_name(err));
       	break;
    }
	return(err);
}

esp_err_t NVS::readNVSu32(nvs_handle aHandle, const char *aKey, uint32_t *aValue)
{
	esp_err_t err = ESP_OK;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		err = nvs_get_u32(aHandle, aKey, aValue);
		xSemaphoreGive(semAction);
	}
    switch (err) {
    case ESP_OK:
      	ESP_LOGD(TAG, "Read %s = %u", aKey, *aValue);
        break;
    case ESP_ERR_NVS_NOT_FOUND:
      	ESP_LOGW(TAG, "Read %s value not Found!", aKey);
      	*aValue = 0;
        break;
    default:
       	ESP_LOGW(TAG, "Error (%s) reading!!", esp_err_to_name(err));
       	break;
    }
	return(err);
}

esp_err_t NVS::readNVSint32Table(nvs_handle aHandle, const char **aKeyVector, int32_t **aValueVector, uint8_t numOfElements)
{
	esp_err_t err = ESP_OK;
	bool alMostError = false;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		for(uint8_t i=0; i<numOfElements; i++) {
			err = nvs_get_i32(aHandle, aKeyVector[i], aValueVector[i]);
			if(err != ESP_OK) alMostError = true;
		}
		xSemaphoreGive(semAction);
	}
	if(alMostError)
		return(ESP_ERR_NVS_NOT_FOUND);
	else
		return(ESP_OK);
}

esp_err_t NVS::writeNVSString(nvs_handle aHandle, const char *aKey, const char *aValue)
{
	esp_err_t err = ESP_OK;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		err = nvs_set_str(aHandle, aKey, aValue);
		if(err == ESP_OK) {
			ESP_LOGD(TAG, "Write %s = %s", aKey, aValue);
			err = nvs_commit(aHandle);
		}
		xSemaphoreGive(semAction);
	}
    if(err != ESP_OK)
    	ESP_LOGW(TAG, "Failed to write %s=%s ! (%s)", aKey, aValue, esp_err_to_name(err));
	return(err);
}

esp_err_t NVS::writeNVSi32(nvs_handle aHandle, const char *aKey, int32_t aValue)
{
	esp_err_t err = ESP_OK;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		err = nvs_set_i32(aHandle, aKey, aValue);
		if(err == ESP_OK) {
			ESP_LOGD(TAG, "Write %s = %d", aKey, aValue);
			err = nvs_commit(aHandle);
		}
		xSemaphoreGive(semAction);
	}
	if(err != ESP_OK)
		ESP_LOGW(TAG, "Failed to write %s=%d ! (%s)", aKey, aValue, esp_err_to_name(err));
	return(err);
}

esp_err_t NVS::writeNVSu32(nvs_handle aHandle, const char *aKey, uint32_t aValue)
{
	esp_err_t err = ESP_OK;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		err = nvs_set_u32(aHandle, aKey, aValue);
		if(err == ESP_OK) {
			ESP_LOGD(TAG, "Write %s = %u", aKey, aValue);
			err = nvs_commit(aHandle);
		}
		if(err != ESP_OK)
			ESP_LOGW(TAG, "Failed to write %s=%d ! (%s)", aKey, aValue, esp_err_to_name(err));
		xSemaphoreGive(semAction);
	}
	return(err);
}

esp_err_t NVS::writeNVSint32Table(nvs_handle aHandle, const char **aKeyVector, int32_t *aValueVector, uint8_t numOfElements)
{
	esp_err_t err = ESP_OK;
	bool alMostError = false;
	if( xSemaphoreTake( semAction, ( TickType_t ) 10 ) == pdTRUE ) {
		for(uint8_t i=0; i<numOfElements; i++) {
			err = nvs_set_i32(aHandle, aKeyVector[i], aValueVector[i]);
			if(err != ESP_OK) alMostError = true;
		}
		err = nvs_commit(aHandle);
		if(err != ESP_OK) alMostError = true;
		xSemaphoreGive(semAction);
	}
 	if(alMostError)
 	    return(ESP_ERR_NVS_CORRUPT_KEY_PART);
 	else
 		return(ESP_OK);
}



