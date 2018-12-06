/*
 * sntp.h
 *
 *  Created on: Nov 30, 2018
 *      Author: fap
 */

#ifndef MAIN_LIB_SNTP_H_
#define MAIN_LIB_SNTP_H_

#include "WiFi.h"

bool initialize_sntp(WiFi *theWifi);
void obtain_time(WiFi *theWifi);



#endif /* MAIN_LIB_SNTP_H_ */
