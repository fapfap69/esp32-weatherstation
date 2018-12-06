/*
 * WiFi.h
 *
 *  Created on: Nov 29, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_WIFI_H_
#define MAIN_LIB_WIFI_H_

#include "sdkconfig.h"

#include <string>
#include <vector>
#include <mdns.h>
#include <esp_err.h>
#include <esp_event_legacy.h>
#include <esp_wifi_types.h>
#include <tcpip_adapter.h>
#include <lwip/ip_addr.h>

#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "Blinker.h"

#define WIFISTATION_BACONSINTERVAL 10

class WiFi {
public:
	static EventGroupHandle_t wifiEG;
	static const int WIFICONNECTED;
	static const int WIFIRUNNING;

protected:
	static ip4_addr_t		ip;
	static ip4_addr_t		gw;
	static ip4_addr_t		netmask;
	static std::string		m_ssid;
	static std::string		m_passwd;
	static wifi_mode_t 		m_mode;
	static bool             m_eventLoopStarted;
	static bool             m_initCalled;
	static uint8_t          m_apConnectionStatus;   // ESP_OK = we are connected to an access point.  Otherwise receives wifi_err_reason_t.
	static Blinker			*blkLed;

private:
	static WiFi* inst_;   // The one, single instance
	static SemaphoreHandle_t semAction;

	WiFi(); // private constructor
	WiFi(const WiFi&);
	WiFi& operator=(const WiFi&);

	static esp_err_t    EventHandler(void* ctx, system_event_t* event);

public:
	static WiFi* getInstance();
	esp_err_t init();
	esp_err_t connectAP();
	esp_err_t connectAP(const std::string& ssid, const std::string& password, wifi_mode_t mode);
	void setAP(const std::string& ssid, const std::string& password);
	void setDNSServer(int numdns, ip_addr_t ip);
	char *getMAC(char* theMAC);
	bool isConnectedToAP();
	void setStaticIP(uint32_t aIp, uint32_t aGatway, uint32_t aNetmask);

	void Sleep();
};



#endif /* MAIN_LIB_WIFI_H_ */
