/*
 * UpdateFirmware.h
 *
 *  Created on: Nov 26, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_UPDATEFIRMWARE_H_
#define MAIN_LIB_UPDATEFIRMWARE_H_

#include <stdio.h>
#include "esp_http_client.h"
#include "freertos/semphr.h"

#include "../Blinker/Blinker.h"

#include "../WiFi/WiFi.h"

#define BUFFSIZE 4096
#define HASH_LEN 32 /* SHA-256 digest length */
#define DEF_APPSERVER_URI "https://192.168.1.1/generic.bin"

#define LOADER_PART_NAME "Loader"
#define APPLICATION_PART_NAME "Applic"

// 250msec wait cycles
#define WIFI_TIMEOUT_CYCLES 400


class UpdateFirmware
{
public:
	static EventGroupHandle_t updateFirmware_EG;
	static const int UPDFRW_NOW;
	static const int UPDFRW_DONE;
	static const int UPDFRW_ERROR;

private:
	static esp_partition_t *partToUpdate;
	static char serverUri[256];
	static bool updateDone;
	static bool updateStart;
	static unsigned long bytesDownloaded;
	static esp_http_client_handle_t client;
	static esp_ota_handle_t update_handle;
	static esp_err_t lastErrCode;

	static uint8_t sha256loader[HASH_LEN];
	static uint8_t sha256application[HASH_LEN];
	static esp_partition_t *partLoader;
	static esp_partition_t *partApplication;
	static esp_partition_t *partRunning;
	static esp_partition_t *partBoot;

	static Blinker	*blkLed;
	static WiFi	*wifi;

	static UpdateFirmware* inst_;   // The one, single instance
	static SemaphoreHandle_t semAction;

public:
	static UpdateFirmware* getInstance();
	esp_err_t Update(const char *aServerUri, const char *aServerCertPem);
	esp_err_t SwitchPartition();
	void Restart();
	esp_err_t SwitchToLoader();

private:
	UpdateFirmware(); // private constructor
	UpdateFirmware(const UpdateFirmware&);
	UpdateFirmware& operator=(const UpdateFirmware&);

private:
	static void http_cleanup(esp_http_client_handle_t client);
	void print_sha256 (const uint8_t *image_hash, const char *label);
	bool checkPartTable();
	static void download(void *);

#ifdef LARGE
	bool checkPartition(esp_partition_t* otaPartition);
#endif

};



#endif /* MAIN_LIB_UPDATEFIRMWARE_H_ */
