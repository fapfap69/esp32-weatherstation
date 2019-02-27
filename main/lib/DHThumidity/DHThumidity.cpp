
/*
DHThumidity.cpp

Author A.Franco
Date 31/01/2013
Ver. 0.1

*/
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include <math.h>
#include <time.h>

#include "DHThumidity.h"

#define MAXdhtData 5	// to complete 40 = 5*8 Bits


static const char* TAG = "DHT";
Blinker *DHThumidity::blkLed = Blinker::getInstance();
// class constructor
DHThumidity::DHThumidity() {

}

// Setup of lines
void DHThumidity::PowerOn(gpio_num_t pin, int type) {
	sensor.PinNumber = pin;
	sensor.Type = (uint8_t)type;
	first = true;
	esp_err_t err;
	err = gpio_set_direction(pin, GPIO_MODE_OUTPUT);
	err = gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
	err = gpio_set_level(pin, 1);
	lastReadTime = 0;
	return;
}

// Interface functions
float DHThumidity::getTemperature(bool isCelsius) {
	if(isCelsius) return(sensor.TemperatureC);
	else return(sensor.TemperatureF);
}

float DHThumidity::getHumidity(void) {
	return(sensor.Humidity);
}

float DHThumidity::getDevPoint(void) {
	float A0, SUM;
	A0 = 373.15/(273.15 + sensor.TemperatureC);
  SUM = -7.90298 * (A0-1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
  SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
  SUM += log10(1013.246);
  float VP = pow(10, SUM-3) * sensor.Humidity;
  float T = log(VP/0.61078);   // temp var
  return (241.88 * T) / (17.558-T);
}

float DHThumidity::getDevPointFast(void) {
  double a = 17.271;
  double b = 237.7;
  double temp = (a * sensor.TemperatureC) / (b + sensor.TemperatureC) + log(sensor.Humidity/100);
  double Td = (b * temp) / (a - temp);
  return Td;
}

// The real aquire function
Humiditysensor DHThumidity::readHumidity(void)
{
	float u,t;
	if (readSensor()) {
		switch (sensor.Type) {
			case DHT11:
				u = dataBuffer[0];
				t = dataBuffer[2];
				break;
			case DHT22:
			case DHT21:
				u = dataBuffer[0];
				u *= 256;
				u += dataBuffer[1];
				u /= 10;
				t = dataBuffer[2] & 0x7F;
				t *= 256;
				t += dataBuffer[3];
				t /= 10;
				if (dataBuffer[2] & 0x80) t *= -1;
				break;
			default:
				u = 0;
				t = 0;
				break;
	  }
	} else {
		logError();
		u = -1;
		t = -1;
	}

	sensor.TemperatureC = t;
	sensor.TemperatureF = convertCtoF(t);
	sensor.Humidity = u;
	return(sensor);
}

// Convert celsius to Farhenait
float DHThumidity::convertCtoF(float c)
{
	return c * 1.8 + 32.0;
}

//Celsius to Kelvin conversion
float DHThumidity::convertCtoK(float c)
{
  return c + 273.15;
}

void DHThumidity::logError()
{
	switch(errorCode) {
		case DHT_TIMEOUT_ERROR :
			ESP_LOGE( TAG, "Sensor Timeout\n" );
			break;
		case DHT_CHECKSUM_ERROR:
			ESP_LOGE( TAG, "CheckSum error\n" );
			break;
		case DHT_OK:
			break;
		default :
			ESP_LOGE( TAG, "Unknown error\n" );
	}
}

/*-------------------------------------------------------------------------------
;
;	get next state
;
;	I don't like this logic. It needs some interrupt blocking / priority
;	to ensure it runs in realtime.
;
;--------------------------------------------------------------------------------*/

int DHThumidity::getSignalLevel( int usTimeOut, bool state )
{
	int uSec = 0;
	while( gpio_get_level(sensor.PinNumber)==state ) {
		if( uSec > usTimeOut )
			return -1;
		++uSec;
		ets_delay_us(1);		// uSec delay
	}
	return uSec;
}

/*----------------------------------------------------------------------------
;
;	read DHT22 sensor
copy/paste from AM2302/DHT22 Docu:
DATA: Hum = 16 bits, Temp = 16 Bits, check-sum = 8 Bits
Example: MCU has received 40 bits data from AM2302 as
0000 0010 1000 1100 0000 0001 0101 1111 1110 1110
16 bits RH data + 16 bits T data + check sum
1) we convert 16 bits RH data from binary system to decimal system, 0000 0010 1000 1100 → 652
Binary system Decimal system: RH=652/10=65.2%RH
2) we convert 16 bits T data from binary system to decimal system, 0000 0001 0101 1111 → 351
Binary system Decimal system: T=351/10=35.1°C
When highest bit of temperature is 1, it means the temperature is below 0 degree Celsius.
Example: 1000 0000 0110 0101, T= minus 10.1°C: 16 bits T data
3) Check Sum=0000 0010+1000 1100+0000 0001+0101 1111=1110 1110 Check-sum=the last 8 bits of Sum=11101110
Signal & Timings:
The interval of whole process must be beyond 2 seconds.
To request data from DHT:
1) Sent low pulse for > 1~10 ms (MILI SEC)
2) Sent high pulse for > 20~40 us (Micros).
3) When DHT detects the start signal, it will pull low the bus 80us as response signal,
   then the DHT pulls up 80us for preparation to send data.
4) When DHT is sending data to MCU, every bit's transmission begin with low-voltage-level that last 50us,
   the following high-voltage-level signal's length decide the bit is "1" or "0".
	0: 26~28 us
	1: 70 us
;----------------------------------------------------------------------------*/


bool DHThumidity::readSensor()
//int readDHT()
{
	int uSec = 0;
	uint8_t byteInx = 0;
	uint8_t bitInx = 7;
	esp_err_t err;

	// clean the buffer
	for (int k = 0; k<MAXdhtData; k++) dataBuffer[k] = 0;

	// Send start signal to DHT sensor
//	err = gpio_set_pull_mode(sensor.PinNumber, GPIO_PULLUP_ONLY);
//	gpio_set_level( sensor.PinNumber, 1 ); //
//	ets_delay_us( 250000 );
	gpio_set_level( sensor.PinNumber, 0 ); // pull down for 3 ms for a smooth and nice wake up
	gpio_set_direction( sensor.PinNumber, GPIO_MODE_OUTPUT );
	ets_delay_us( 3000 );
	gpio_set_direction( sensor.PinNumber, GPIO_MODE_INPUT );		// change to input mode
	gpio_set_level( sensor.PinNumber, 1 ); // pull up for 25 us for a gentile asking for data
//	ets_delay_us( 43 );
//	gpio_set_level( sensor.PinNumber, 0 );

	//  DHT will keep the line low for 80 us and then high for 80us
//	ets_delay_us( 10 );

	int usTimeOut = 2000;
	int samples = 3+40*2;
	int duration[85];
	int s = 0;
	int state = 1;
	duration[s] = 0;

	vTaskSuspendAll();
	while(s<samples && duration[s] < usTimeOut) {
		while( gpio_get_level(sensor.PinNumber)==state && duration[s] < usTimeOut ) {
			++duration[s];
			ets_delay_us(1);		// uSec delay
		}
		state = (state == 0) ? 1:0;
		s++;
		duration[s] = 0;
	}
	xTaskResumeAll();

	printf( "Prima terna  H=%d L=%d H=%d \n", duration[0], duration[1],duration[2]);
	for(int i=3;i<83; i += 2) {
		if (duration[i+1] > 40) { // add the current read to the output data
			dataBuffer[ byteInx ] |= (1 << bitInx);
		}
		// move the pointers
		if (bitInx == 0) { bitInx = 7; ++byteInx; }
		else bitInx--;
		printf( "0=%d 1=%d, ",duration[i],duration[i+1] );
	}
/*
	errorCode = DHT_OK;
	uSec = getSignalLevel( 85, 0 );
	if( uSec<0 ) {
		blkLed->SetPat(BKS_ERROR_SENS);
		errorCode = DHT_TIMEOUT_ERROR;
		printf("p1\n");
	} else {
		uSec = getSignalLevel( 85, 1 );
		if( uSec<0 ) {
			blkLed->SetPat(BKS_ERROR_SENS);
			errorCode = DHT_TIMEOUT_ERROR;
			printf("p2\n");
		} else {
			// Read the 40 data bits
			for( int k = 0; k < 40; k++ ) {
				uSec = getSignalLevel( 60, 0 ); // -- starts new data transmission with >50us low signal
				if( uSec<0 ) {
					blkLed->SetPat(BKS_ERROR_SENS);
					errorCode = DHT_TIMEOUT_ERROR;
					printf("p3\n");
					break;
				}
				uSec = getSignalLevel( 90, 1 ); // -- check to see if after >70us rx data is a 0 or a 1
				if( uSec<0 ) {
					blkLed->SetPat(BKS_ERROR_SENS);
					errorCode = DHT_TIMEOUT_ERROR;
					printf("p4 %d\n",k);
					break;
				}
				if (uSec > 40) { // add the current read to the output data
					dataBuffer[ byteInx ] |= (1 << bitInx);
				}
				// move the pointers
				if (bitInx == 0) { bitInx = 7; ++byteInx; }
				else bitInx--;
			}
		}
	}
//	xTaskResumeAll();
*/
	printf("Check sum %d -> %d\n",dataBuffer[4], ((dataBuffer[0] + dataBuffer[1] + dataBuffer[2] + dataBuffer[3]) & 0xFF));
	if (errorCode == DHT_OK) {
		// Checksum is the sum of Data 8 bits masked out 0xFF
		if (dataBuffer[4] == ((dataBuffer[0] + dataBuffer[1] + dataBuffer[2] + dataBuffer[3]) & 0xFF)) {
			time(&lastReadTime);
		} else {
			blkLed->SetPat(BKS_ERROR_SENS);
			errorCode = DHT_CHECKSUM_ERROR;
			printf("p5\n");
		}
	}
	// Switch Off
	gpio_set_direction( sensor.PinNumber, GPIO_MODE_OUTPUT );
//	gpio_set_level( sensor.PinNumber, 0 ); // pull down for 3 ms for a smooth and nice wake up
//	ets_delay_us( 3 );
	gpio_set_level( sensor.PinNumber, 1 ); // pull down for 3 ms for a smooth and nice wake up

//	blkLed->SetPat(BKS_ERROR_SENS);
	return errorCode;
}
