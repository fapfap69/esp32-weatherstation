/*
 * mQTTclient.h
 *
 *  Created on: Nov 26, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_MQTTCLIENT_H_
#define MAIN_LIB_MQTTCLIENT_H_

#define DEF_MQTT_BROKER_URL		"mqtt://127.0.0.1:1883"
#define DEF_MQTT_STATION_NAME	"test"

#define BROKER_NAME_LEN	127
#define STATION_NAME_LEN 63
#define ITEM_NAME_LEN 256

#define MQTT_TIMEOUT_MILLISEC 100

#define QOS_ATMOST_ONCE	0
#define QOS_ATLEAST_ONCE	1
#define QOS_EXACTLY_ONCE	2

#define DATA_NORETAINED	false
#define DATA_RETAINED	true

#define CONNECTED_BIT	BIT(0)
#define PUBLISH_BIT	BIT(1)
#define SUBSCRIBE_BIT	BIT(1)

#define INVALID_PUBLICATION -1
#define INVALID_SUBCRIPTION -1

#define MQTTCL_LEN_BROKER	256
#define MQTTCL_LEN_STATION	256

#include "sdkconfig.h"

#include <vector>
#include <string>

#include <esp_err.h>
#include <esp_log.h>

#include <esp_event.h>
#include <esp_event_loop.h>
#include <esp_event_legacy.h>
#include <mqtt_client.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "../Blinker/Blinker.h"

class mQttPubList {
public:
	struct publishedItem {
		std::string theValue;
		std::string theName;
		int dataLen;
		bool isChanged;
		bool isPublished;
	};
	typedef std::vector<publishedItem> mQtt_pubList;
private:
	mQtt_pubList theList;
public:
	mQttPubList();
	~mQttPubList();
	void AddItem(const char* ItemName, const char* Value, uint8_t datalen = 0);
	void AddItem(const char* ItemName, int Value);
	void AddItem(const char* ItemName, float Value);
	uint size() {return(theList.size()) ; }
	bool isToPublish(uint index);
	publishedItem *GetItem(uint index);
};




class mQttClient {
private:

	typedef esp_err_t (*mqttclient_event_cb_t)(esp_mqtt_event_handle_t event, void *cx);
	struct subscribedItem {
		mqttclient_event_cb_t theEventHandler;
		std::string theName;
	};

public:
	static EventGroupHandle_t mqttEG;
	static const int MQTTCONNECTED;
	static const int MQTTPUBLISHING;
	static const int MQTTSUBSCRIBE;

private:
	static std::vector<subscribedItem> ssItems;
	static Blinker	*blkLed;
	static bool Start;

	static int QoS;
	static bool Retain;
	static char brokerUri[BROKER_NAME_LEN+1];
	static char baseName[STATION_NAME_LEN+1];
	static esp_mqtt_client_config_t mqtt_cfg;
	static esp_mqtt_client_handle_t client;
	static int publishedMessageId;
	static int subscribedMessageId;

	static TickType_t xTicksToWait;
	static mQttClient* inst_;   // The one, single instance
	static SemaphoreHandle_t semAction;

public:
	static mQttClient* getInstance();

	char *getBrokerUri();
	void setBrokerUri(const char* aUri);
	char *getDeviceName();
	void setDeviceName(const char* aName);
	int getQoS() { return QoS; };
	void setQoS(int aQoS) { QoS = aQoS; return; };
	bool getRetained() { return Retain; };
	void setRetained(bool aRetained) { Retain = aRetained; return; };
	int getTimeout() { return((int)xTicksToWait * portTICK_PERIOD_MS); };
	void setTimeout(int millisec) { xTicksToWait = (TickType_t)(millisec / portTICK_PERIOD_MS); return; };

	void Stop();
	void init();

	int Publication(const char* ItemName, int Data);
	int Publication(const char* ItemName, float Data);
	int Publication(const char* ItemName, const char* Data, int dataLen);
	int Publication(mQttPubList *aListOfPublications);

	int Subscribe(const char* ItemName, mqttclient_event_cb_t cbFunction);
	int Unsubscribe(const char* ItemName);

private:
	mQttClient();
	mQttClient(const mQttClient&);
	mQttClient& operator=(const mQttClient&);

	static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);
	void redoSubscriptions();


};


#endif /* MAIN_LIB_MQTTCLIENT_H_ */
