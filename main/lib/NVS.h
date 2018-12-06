/*
 * nvsUtility.h
 *
 *  Created on: Nov 29, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_NVS_H_
#define MAIN_LIB_NVS_H_

#include "esp_system.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "sdkconfig.h"

#include "Blinker.h"

class NVS
{
private:
	static nvs_handle generalNVS;
	static nvs_handle perdeviceNVS;
	static const char *TAG;
	static Blinker	*blkLed;
	static NVS* inst_;   // The one, single instance
	static SemaphoreHandle_t semAction;


public:
	static NVS* getInstance();
	bool Init(void);
	void Close();

	esp_err_t rStr_NVS(const char *aKey, char *aValue, size_t maxLen) {return(readNVSString(generalNVS, aKey, aValue, maxLen));}
	esp_err_t rInt_NVS(const char *aKey, int32_t *aValue) {return(readNVSi32(generalNVS, aKey, aValue));}
	esp_err_t rUint_NVS(const char *aKey, uint32_t *aValue) {return(readNVSu32(generalNVS, aKey, aValue));}
	esp_err_t rIntTable_NVS(const char **aKey, int32_t **aValue, uint8_t num) {return(readNVSint32Table(generalNVS, aKey, aValue, num));}
	esp_err_t rStr_Dev(const char *aKey, char *aValue, size_t maxLen) {return(readNVSString(perdeviceNVS, aKey, aValue, maxLen));}
	esp_err_t rInt_Dev(const char *aKey, int32_t *aValue) {return(readNVSi32(perdeviceNVS, aKey, aValue));}
	esp_err_t rUint_Dev(const char *aKey, uint32_t *aValue) {return(readNVSu32(perdeviceNVS, aKey, aValue));}
	esp_err_t rIntTable_Dev(const char **aKey, int32_t **aValue, uint8_t num) {return(readNVSint32Table(perdeviceNVS, aKey, aValue, num));}

	esp_err_t wStr_NVS(const char *aKey, const char *aValue) {return(writeNVSString(generalNVS, aKey, aValue));}
	esp_err_t wInt_NVS(const char *aKey, int32_t aValue) {return(writeNVSi32(generalNVS, aKey, aValue));}
	esp_err_t wUint_NVS(const char *aKey, uint32_t aValue) {return(writeNVSu32(generalNVS, aKey, aValue));}
	esp_err_t wIntTable_NVS(const char **aKey, int32_t *aValue, uint8_t num) {return(writeNVSint32Table(generalNVS, aKey, aValue, num));}
	esp_err_t wStr_Dev(const char *aKey, const char *aValue) {return(writeNVSString(generalNVS, aKey, aValue));}
	esp_err_t wInt_Dev(const char *aKey, int32_t aValue) {return(writeNVSi32(generalNVS, aKey, aValue));}
	esp_err_t wUint_Dev(const char *aKey, uint32_t aValue) {return(writeNVSu32(generalNVS, aKey, aValue));}
	esp_err_t wIntTable_Dev(const char **aKey, int32_t *aValue, uint8_t num) {return(writeNVSint32Table(perdeviceNVS, aKey, aValue, num));}


private:
	NVS();
	NVS(const NVS&);
	NVS& operator=(const NVS&);

	static esp_err_t readNVSString(nvs_handle aHandle, const char *aKey, char *aValue, size_t maxLen);
	static esp_err_t readNVSi32(nvs_handle aHandle, const char *aKey, int32_t *aValue);
	static esp_err_t readNVSu32(nvs_handle aHandle, const char *aKey, uint32_t *aValue);
	static esp_err_t readNVSint32Table(nvs_handle aHandle, const char **aKeyVector, int32_t **aValueVector, uint8_t numOfElements);

	static esp_err_t writeNVSString(nvs_handle aHandle, const char *aKey, const char *aValue);
	static esp_err_t writeNVSi32(nvs_handle aHandle, const char *aKey, int32_t aValue);
	static esp_err_t writeNVSu32(nvs_handle aHandle, const char *aKey, uint32_t aValue);
	static esp_err_t writeNVSint32Table(nvs_handle aHandle, const char **aKey, int32_t *aValue, uint8_t numOfElements);

};

#endif /* MAIN_LIB_NVS_H_ */
