/*
 * mQttClient.cpp
 *
 *  Created on: Nov 26, 2018
 *      Author: fap
 */
#include "mQttClient.h"

#include <vector>
#include <stdio.h>
#include <string.h>

#include <esp_log.h>

#include "../GeneralUtils/GeneralUtils.h"

#define TAG "MQTT"

mQttPubList::mQttPubList()
{
	theList.clear();
}
mQttPubList::~mQttPubList(){
//	delete theList;
}
void mQttPubList::AddItem(const char*ItemName, const char* Value, uint8_t datalen)
{
	publishedItem *appo = NULL;
	std::string theStrinBuffer;
	for(int i=0;i<theList.size(); i++){
		if(theList.at(i).theName == ItemName) {
			appo = &theList.at(i);
			break;
		}
	}
	if(appo == NULL) {
		appo = new publishedItem;
		theList.push_back(*appo);
		appo = &theList.at(theList.size()-1);
	}
	if(datalen == 0){
		appo->theValue = Value;
	} else {
		GeneralUtils::base64Encode(Value, &theStrinBuffer);
		appo->theValue = theStrinBuffer;
	}
	appo->dataLen = strlen(appo->theValue.c_str());
	appo->isChanged = true;
	appo->isPublished = false;
	return;
}
void mQttPubList::AddItem(const char*ItemName, int Value)
{
	char buffer[32];
	sprintf(buffer,"%d",Value);
	AddItem(ItemName,buffer);
	return;
}
void mQttPubList::AddItem(const char*ItemName, float Value)
{
	char buffer[32];
	sprintf(buffer,"%f",Value);
	AddItem(ItemName,buffer);
	return;
}

bool mQttPubList::isToPublish(uint index)
{
	if(index >= theList.size()) return(false);
	return( (theList.at(index).isChanged && !theList.at(index).isPublished) ? true : false);
}
mQttPubList::publishedItem *mQttPubList::GetItem(uint index)
{
	if(index >= theList.size()) return(NULL);
	return( &theList.at(index) );
}


std::vector<mQttClient::subscribedItem> mQttClient::ssItems;

EventGroupHandle_t mQttClient::mqttEG;
const int mQttClient::MQTTCONNECTED = BIT0;
const int mQttClient::MQTTPUBLISHING = BIT1;
const int mQttClient::MQTTSUBSCRIBE = BIT2;

bool mQttClient::Start = false;
int mQttClient::QoS = QOS_ATLEAST_ONCE;
bool mQttClient::Retain = DATA_RETAINED;
int mQttClient::publishedMessageId = INVALID_PUBLICATION;
int mQttClient::subscribedMessageId = INVALID_SUBCRIPTION;

esp_mqtt_client_config_t mQttClient::mqtt_cfg;
esp_mqtt_client_handle_t mQttClient::client = NULL;

char mQttClient::brokerUri[BROKER_NAME_LEN+1] = DEF_MQTT_BROKER_URL;
char mQttClient::baseName[STATION_NAME_LEN+1] = DEF_MQTT_STATION_NAME;
char mQttClient::user[BROKER_NAME_LEN+1] = DEF_MQTT_USER;
char mQttClient::passwd[BROKER_NAME_LEN+1] = DEF_MQTT_PASSWD;

TickType_t mQttClient::xTicksToWait = MQTT_TIMEOUT_MILLISEC / portTICK_PERIOD_MS;
mQttClient *mQttClient::inst_ = NULL;   // The one, single instance
SemaphoreHandle_t mQttClient::semAction = NULL;

Blinker *mQttClient::blkLed = NULL;


mQttClient* mQttClient::getInstance() {
   if (inst_ == NULL) {
      inst_ = new mQttClient();
   }
   return(inst_);
}


mQttClient::mQttClient()
{
	blkLed = Blinker::getInstance();
	mqttEG = xEventGroupCreate();
	xEventGroupClearBits(mqttEG, MQTTCONNECTED | MQTTPUBLISHING);
	semAction = xSemaphoreCreateMutex();
	if( semAction == NULL ) {
		ESP_LOGE(TAG, "Error to create semaphore !");
	}
}

char *mQttClient::getBrokerUri()
{
	return brokerUri;
}
void mQttClient::setBrokerUri(const char* aUri)
{
	strncpy(brokerUri, aUri, BROKER_NAME_LEN);
	return;
}
char *mQttClient::getDeviceName()
{
	return baseName;
}
void mQttClient::setDeviceName(const char* aName)
{
	strcpy(baseName,"/");
	strncat(baseName, aName, STATION_NAME_LEN);
	strncat(baseName, "/", STATION_NAME_LEN);
	return;
}

void mQttClient::setUserAccount(const char* aUser, const char *aPassw)
{
	strncpy(user,aUser, BROKER_NAME_LEN);
	strncpy(passwd,aPassw, BROKER_NAME_LEN);
	return;
}

void mQttClient::Stop()
{
	ESP_LOGD(TAG, "Stop the mQtt client...");
	if( xSemaphoreTake( semAction, ( TickType_t ) 10000 ) == pdTRUE ) {
		Start = false;
		ESP_LOGD(TAG, "mQtt client stopping...");
		esp_mqtt_client_destroy(client);
		client = NULL;
		ESP_LOGI(TAG, "mQtt client stopped !");
		xSemaphoreGive( semAction );
	} else {
		ESP_LOGW(TAG, "Unable to stop the mQtt client");
	}
    return;
}

void mQttClient::init()
{
	mqtt_cfg.uri = brokerUri;
	mqtt_cfg.event_handle = mqtt_event_handler;
	// .port
	// .cert_pem(const char *)
    // .user_context = (void *)your_context
	mqtt_cfg.username = user;
	mqtt_cfg.password = passwd;
    Start = true;
	client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
    redoSubscriptions();

    return;
}


esp_err_t mQttClient::mqtt_event_handler(esp_mqtt_event_handle_t event)
{
	esp_err_t err;
    esp_mqtt_client_handle_t client = event->client;
    char topic[250];
   // char data[250];

    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        	xEventGroupSetBits(mqttEG, MQTTCONNECTED);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            xEventGroupClearBits(mqttEG, MQTTCONNECTED);
            if(Start) esp_mqtt_client_start(client);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT_EVENT_ERROR");
            blkLed->SetPat(BKS_ERROR_MQTT);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            xEventGroupSetBits(mqttEG, MQTTPUBLISHING);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGD(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            xEventGroupSetBits(mqttEG, MQTTSUBSCRIBE);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGD(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            xEventGroupSetBits(mqttEG, MQTTSUBSCRIBE);
            break;

        case MQTT_EVENT_DATA:
        	memcpy(topic,event->topic,event->topic_len);
        	topic[event->topic_len] = '\0';

            ESP_LOGD(TAG, "MQTT_EVENT_RECEICEDATA %s=%s %d",topic, event->data, event->data_len);
    		for (unsigned i=0; i < ssItems.size(); i++) {
    			if (ssItems[i].theName == topic) {
    				if(ssItems[i].theEventHandler != NULL) {
    					err = ssItems[i].theEventHandler(event, NULL);
    					ESP_LOGD(TAG, "MQTT_EVENT_RECEICEDATA Execute CB fun (%s->%10s)",topic,event->data);
    					break;
    				}
    			}
    		}
    		ESP_LOGW(TAG, "MQTT_EVENT_RECEICEDATA no Call Back function executed !");
            break;

    }
    return ESP_OK;
}

int mQttClient::Publication(const char* ItemName, const char* Data, int dataLen)
{
	EventBits_t uxBits;
	int msg_id;
	char name[ITEM_NAME_LEN];
	if( xSemaphoreTake( semAction, ( TickType_t ) 100 ) == pdTRUE ) {
		uxBits = xEventGroupWaitBits(mqttEG, MQTTCONNECTED, false, true, 1); // no wait just get the bits
		if ((uxBits & MQTTCONNECTED) != MQTTCONNECTED) {
			ESP_LOGW(TAG, "Error mQTT client not connected !");
			publishedMessageId = INVALID_PUBLICATION;
		} else {
			strcpy(name,baseName);
			strcat(name,ItemName);
			xEventGroupClearBits(mqttEG, MQTTPUBLISHING);
			msg_id = esp_mqtt_client_publish(client, name, Data, dataLen, QoS, Retain);
			uxBits = xEventGroupWaitBits(mqttEG, MQTTCONNECTED | MQTTPUBLISHING , false, true, xTicksToWait);
			if ((uxBits & MQTTPUBLISHING) == MQTTPUBLISHING) {
				ESP_LOGD(TAG, "sent publish '%s' successful, msg_id=%d",name, msg_id);
				ESP_LOGD(TAG, "		Data := '%s' ",Data);
				publishedMessageId = msg_id;
			} else {
				ESP_LOGW(TAG, "Error to publish '%s', msg_id=%d",name, msg_id);
				publishedMessageId = INVALID_PUBLICATION;
			}
		}
		xSemaphoreGive(semAction);
	}
	return publishedMessageId;
}

int mQttClient::Publication(const char* ItemName, int Data)
{
	char Buffer[20];
	sprintf(Buffer,"%d",Data);
	return Publication(ItemName, Buffer, 0);
}
int mQttClient::Publication(const char* ItemName, float Data)
{
	char Buffer[20];
	sprintf(Buffer,"%f",Data);
	return Publication(ItemName, Buffer, 0);
}

int mQttClient::Publication(mQttPubList *aListOfPublications)
{
	int msg_id;
	int counter = 0;
	if( xSemaphoreTake( semAction, ( TickType_t ) 100 ) == pdTRUE ) {
		for(int i=0;i<aListOfPublications->size();i++) {
			if(aListOfPublications->isToPublish(i)) {
				mQttPubList::publishedItem *theItem = aListOfPublications->GetItem(i);
				xEventGroupClearBits(mqttEG, MQTTPUBLISHING);
				msg_id = esp_mqtt_client_publish(client, theItem->theName.c_str(), theItem->theValue.c_str(), 0, QoS, Retain);
				xEventGroupWaitBits(mqttEG, MQTTPUBLISHING , false, true, xTicksToWait);
				counter++;
			}
		}
		xSemaphoreGive(semAction);
	}
	return(counter);
}

// ------------------

int mQttClient::Subscribe(const char* ItemName, mqttclient_event_cb_t cbFunction)
{
	EventBits_t uxBits;
	int msg_id;
	char name[ITEM_NAME_LEN];
	subscribedItem theItem;

	if( xSemaphoreTake( semAction, ( TickType_t ) 100 ) == pdTRUE ) {
		uxBits = xEventGroupWaitBits(mqttEG, MQTTCONNECTED, false, true, 1); // no wait just get the bits
		if ((uxBits & MQTTCONNECTED) != MQTTCONNECTED) {
			ESP_LOGW(TAG, "Error mQTT client not connected !");
			subscribedMessageId = INVALID_SUBCRIPTION;
		} else {
			strcpy(name,baseName);
			strcat(name,ItemName);
			xEventGroupClearBits(mqttEG, MQTTSUBSCRIBE);
			msg_id = esp_mqtt_client_subscribe(client, name, QoS);
			uxBits = xEventGroupWaitBits(mqttEG, MQTTCONNECTED | MQTTSUBSCRIBE , false, true, xTicksToWait);
			if ((uxBits & MQTTSUBSCRIBE) == MQTTSUBSCRIBE) {
				ESP_LOGD(TAG, "sent subscribe '%s' successful, msg_id=%d",name, msg_id);
				subscribedMessageId = msg_id;
				theItem.theName = name;
				theItem.theEventHandler = cbFunction;
				ssItems.push_back(theItem);
			} else {
				ESP_LOGW(TAG, "Error to subscribe '%s', msg_id=%d",name, msg_id);
				subscribedMessageId = INVALID_SUBCRIPTION;
			}
		}
		xSemaphoreGive(semAction);
	}
	return subscribedMessageId;
}

void mQttClient::redoSubscriptions()
{
	int msg_id;
	if( xSemaphoreTake( semAction, ( TickType_t ) 100 ) == pdTRUE ) {
		for(int i=0;i<ssItems.size();i++) {
			xEventGroupClearBits(mqttEG, MQTTSUBSCRIBE);
			msg_id = esp_mqtt_client_subscribe(client, ssItems.at(i).theName.c_str(), QoS);
			xEventGroupWaitBits(mqttEG, MQTTSUBSCRIBE , false, true, xTicksToWait);
		}
		xSemaphoreGive(semAction);
	}
	return;
}


int mQttClient::Unsubscribe(const char* ItemName)
{
	int msg_id;
	EventBits_t uxBits;
	char name[ITEM_NAME_LEN];

	strcpy(name,baseName);
	strcat(name,ItemName);
	if( xSemaphoreTake( semAction, ( TickType_t ) 100 ) == pdTRUE ) {
		xEventGroupClearBits(mqttEG, MQTTSUBSCRIBE);
		msg_id = esp_mqtt_client_unsubscribe(client, name);
		uxBits = xEventGroupWaitBits(mqttEG, MQTTCONNECTED | MQTTSUBSCRIBE , false, true, xTicksToWait);
		if ((uxBits & MQTTSUBSCRIBE) == MQTTSUBSCRIBE) {
			for (unsigned i=0; i < ssItems.size(); i++) {
				if (ssItems[i].theName == name) {
					ssItems.erase(ssItems.begin() + i);
				}
			}
			ESP_LOGD(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
			subscribedMessageId = msg_id;
		} else {
			ESP_LOGW(TAG, "Error to subscribe '%s', msg_id=%d",name, msg_id);
			subscribedMessageId = INVALID_SUBCRIPTION;
		}
		xSemaphoreGive(semAction);
	}
	return subscribedMessageId;
}
