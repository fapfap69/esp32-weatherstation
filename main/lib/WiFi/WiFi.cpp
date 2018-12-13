/*
 * WiFi.cpp
 *
 *  Created on: Nov 29, 2018
 *      Author: fap
 */
#include "WiFi.h"

#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include "sdkconfig.h"


#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <nvs_flash.h>
#include <lwip/dns.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>



#include <string.h>

#include "../GeneralUtils/GeneralUtils.h"

static const char* TAG = "WiFi";

const int WiFi::WIFICONNECTED = BIT0;
const int WiFi::WIFIRUNNING = BIT1;

Blinker *WiFi::blkLed = Blinker::getInstance();
EventGroupHandle_t WiFi::wifiEG = 0;
ip4_addr_t		WiFi::ip = {.addr = 0};
ip4_addr_t		WiFi::gw = {.addr = 0};
ip4_addr_t		WiFi::netmask = {.addr = 0};
std::string		WiFi::m_ssid = "";
std::string		WiFi::m_passwd = "";
wifi_mode_t 	WiFi::m_mode = WIFI_MODE_STA;
bool            WiFi::m_eventLoopStarted = false;
bool            WiFi::m_initCalled =false;
uint8_t         WiFi::m_apConnectionStatus = UINT8_MAX;   // ESP_OK = we are connected to an access point.  Otherwise receives wifi_err_reason_t.
WiFi			*WiFi::inst_ = NULL;   // The one, single instance
SemaphoreHandle_t WiFi::semAction = NULL;

WiFi* WiFi::getInstance() {
   if (inst_ == NULL) {
      inst_ = new WiFi();
   }
   return(inst_);
}

WiFi::WiFi()
{
	wifiEG = xEventGroupCreate();
	xEventGroupClearBits(wifiEG, WIFICONNECTED | WIFIRUNNING);
	semAction = xSemaphoreCreateMutex();
	if( semAction == NULL ) {
		ESP_LOGE(TAG, "Error to create semaphore !");
	}
} // WiFi

void WiFi::setStaticIP(uint32_t aIp, uint32_t aGatway, uint32_t aNetmask)
{
	ip.addr = aIp;
	gw.addr = aGatway;
	netmask.addr = aNetmask;
	return;
}

void WiFi::setAP(const std::string& ssid, const std::string& password)
{
	m_ssid = ssid;
	m_passwd = password;
	return;
}

esp_err_t WiFi::connectAP(const std::string& ssid, const std::string& password, wifi_mode_t mode)
{
	esp_err_t err = ESP_OK;
	m_ssid = ssid;
	m_passwd = password;
	m_mode = mode;
	err = init();
	err = configure();
	if(err == ESP_OK)
		err = connectAP();
	return(err);
}

esp_err_t WiFi::connectAP()
{
	esp_err_t errRc = 0;
	if( xSemaphoreTake( semAction, ( TickType_t ) 1000 ) == pdTRUE ) {

		ESP_LOGI(TAG, "Starting driver ...");
		m_apConnectionStatus = UINT8_MAX;

		errRc = ::esp_wifi_start();
		if (errRc == ESP_OK) {
			ESP_LOGD(TAG, " started. wait for connection to AP");
		} else {
			ESP_LOGE(TAG, "Start driver error : rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			blkLed->SetPat(BKS_ERROR_WIFI);
		}
		m_apConnectionStatus = errRc;
		xSemaphoreGive( semAction );
	}
	return m_apConnectionStatus;  // Return ESP_OK if we are now connected and wifi_err_reason_t if not.
} // connectAP


void WiFi::Sleep()
{
	/*
    WIFI_PS_NONE,        < No power save
    WIFI_PS_MIN_MODEM,   < Minimum modem power saving. In this mode, station wakes up to receive beacon every DTIM period
    WIFI_PS_MAX_MODEM,   < Maximum modem power saving. In this mode, interval to receive beacons is determined by the listen_interval parameter in wifi_sta_config_t
	 */
	esp_err_t err;
	wifi_ps_type_t ps;
	err = esp_wifi_get_ps(&ps);

	wifi_config_t sta_config;
	err = esp_wifi_get_config(WIFI_IF_STA, &sta_config);
	sta_config.sta.listen_interval = WIFISTATION_BACONSINTERVAL;
	err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);

	err = esp_wifi_set_ps(WIFI_PS_MAX_MODEM);

	ESP_LOGI(TAG, "Stop the WiFi...");

	err = esp_wifi_stop();
	if(err != ESP_OK) {
		ESP_LOGW(TAG, "Could not stop WiFi: rc=%d %s", err, GeneralUtils::errorToString(err));
    }
	return;
}
esp_err_t WiFi::configure() {
	esp_err_t errRc;

	errRc = ::esp_wifi_set_mode(m_mode);
	if (errRc != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_set_mode: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
		blkLed->SetPat(BKS_ERROR_WIFI);
		return(errRc);
	}
	wifi_config_t sta_config;
	::memset(&sta_config, 0, sizeof(sta_config));
	::memcpy(sta_config.sta.ssid, m_ssid.data(), m_ssid.size());
	::memcpy(sta_config.sta.password, m_passwd.data(), m_passwd.size());
	sta_config.sta.bssid_set = 0;
	errRc = ::esp_wifi_set_config(WIFI_IF_STA, &sta_config);
	if (errRc != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_set_config: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
		blkLed->SetPat(BKS_ERROR_WIFI);
		return(errRc);
	}

	if (ip.addr != 0 && gw.addr != 0 && netmask.addr != 0) {
		::tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA); // Don't run a DHCP client
		tcpip_adapter_ip_info_t ipInfo;
		ipInfo.ip.addr = ip.addr;
		ipInfo.gw.addr = gw.addr;
		ipInfo.netmask.addr = netmask.addr;
		::tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
	}
	ESP_LOGI(TAG, "Wifi Configure done ");
	return(ESP_OK);
}



esp_err_t WiFi::init() {
	esp_err_t errRc = 0;

	if (!m_initCalled) {
		::tcpip_adapter_init();

		if (m_eventLoopStarted) {
			esp_event_loop_set_cb(WiFi::EventHandler, NULL);   // Returns the old handler.
		} else {
			errRc = ::esp_event_loop_init(WiFi::EventHandler, NULL);  // Initialze the event handler.
			if (errRc != ESP_OK) {
				ESP_LOGE(TAG, "esp_event_loop_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
				blkLed->SetPat(BKS_ACTIVE_OK);
				return(errRc);
			}
			m_eventLoopStarted = true;
		}
		// Now, one way or another, the event handler is WiFi::eventHandler.

		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		errRc = ::esp_wifi_init(&cfg);
		if (errRc != ESP_OK) {
			ESP_LOGE(TAG, "esp_wifi_init: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			blkLed->SetPat(BKS_ERROR_WIFI);
			return(errRc);
		}

		errRc = ::esp_wifi_set_storage(WIFI_STORAGE_RAM);
		if (errRc != ESP_OK) {
			ESP_LOGE(TAG, "esp_wifi_set_storage: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
			blkLed->SetPat(BKS_ERROR_WIFI);
			return(errRc);
		}
	}
	ESP_LOGI(TAG, "Wifi Init done ");
	m_initCalled = true;
	return(ESP_OK);
} // init


esp_err_t WiFi::EventHandler(void *ctx, system_event_t *event)
{
	esp_err_t err;
	//WiFi* pWiFi = (WiFi*) ctx;   // retrieve the WiFi object from the passed in context.
	ip_addr_t dnsip;

    switch (event->event_id) {

    case SYSTEM_EVENT_STA_START:
    	m_apConnectionStatus = UINT8_MAX;
		ESP_LOGD(TAG, "Start the Wifi: connecting...");
		err = ::esp_wifi_connect();
    	if(err != ESP_OK) {
    		ESP_LOGW(TAG, "Connection error: rc=%d %s", err, GeneralUtils::errorToString(err));
			blkLed->SetPat(BKS_ERROR_WIFI);
			return(ESP_ERR_WIFI_NOT_CONNECT);
		}
        break;

    case SYSTEM_EVENT_STA_CONNECTED:
		ESP_LOGI(TAG, "Got connection !");
    	xEventGroupSetBits(wifiEG, WIFICONNECTED);
    	break;

    case SYSTEM_EVENT_STA_STOP:
    	m_apConnectionStatus = UINT8_MAX;
    	xEventGroupClearBits(wifiEG, WIFIRUNNING | WIFICONNECTED);
    	break;


    case SYSTEM_EVENT_STA_GOT_IP:
    	tcpip_adapter_ip_info_t ipInfo;
  		::tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ipInfo);
  		ip.addr = ipInfo.ip.addr;
  		gw.addr = ipInfo.gw.addr;
  		netmask.addr = ipInfo.netmask.addr;
  		tcpip_adapter_dns_info_t dns;
  		::tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &dns);
  		dnsip = dns.ip;
  		ESP_LOGI(TAG, "DNS SERVER  %d.%d.%d.%d\n" , ((uint8_t*) (&dnsip))[0], ((uint8_t*) (&dnsip))[1], ((uint8_t*) (&dnsip))[2], ((uint8_t*) (&dnsip))[3]);
        xEventGroupSetBits(wifiEG, WIFIRUNNING);
        m_apConnectionStatus = ESP_OK;
		blkLed->SetPat(BKS_ACTIVE_OK);
        break;

    case SYSTEM_EVENT_STA_LOST_IP:
  		ip.addr = 0;
  		gw.addr = 0;
  		netmask.addr = 0;
  		m_apConnectionStatus = event->event_info.disconnected.reason;
  		xEventGroupClearBits(wifiEG, WIFIRUNNING);
    	break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
    	m_apConnectionStatus = event->event_info.disconnected.reason;
    	xEventGroupClearBits(wifiEG, WIFICONNECTED);
		err = ::esp_wifi_connect();
    	if(err != ESP_OK) {
    		ESP_LOGW(TAG, "Auto connection error: rc=%d %s", err, GeneralUtils::errorToString(err));
			blkLed->SetPat(BKS_ERROR_WIFI);
			return(m_apConnectionStatus);
		}
        m_apConnectionStatus = ESP_OK;
        break;

    default:
        break;
    }
    return m_apConnectionStatus;
}

void WiFi::setDNSServer(int numdns, ip_addr_t ip) {
	ESP_LOGD(TAG, "Setting DNS to %d.%d.%d.%d", ((uint8_t*) (&ip))[0], ((uint8_t*) (&ip))[1], ((uint8_t*) (&ip))[2], ((uint8_t*) (&ip))[3]);
	init();
	::dns_setserver(numdns, &ip);
} // setDNSServer

bool WiFi::isConnectedToAP() {
	return (m_apConnectionStatus == ESP_OK ? true: false);
} // isConnected

char *WiFi::getMAC(char* theMAC) {
	uint8_t mac[6];
	esp_wifi_get_mac(WIFI_IF_STA, mac);
	sprintf(theMAC, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return (theMAC);
}

