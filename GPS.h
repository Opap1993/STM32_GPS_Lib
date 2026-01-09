/*
 * GPS.h
 *
 *  Created on: Jan 9, 2026
 *      Author: John
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wlxx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define DMA_BUF_SIZE 512

typedef struct {
	float decimalLong;
	float decimalLat;
	uint8_t gpsUpdated;
	uint8_t gpsHasFix;
	UART_HandleTypeDef *huart;
} GPS_Data_t;


void GPS_Init(GPS_Data_t *gps, UART_HandleTypeDef *huart);
void  GPS_Process(GPS_Data_t *gps);
void GPS_ProcessCallback(GPS_Data_t *gps, uint16_t Size);
float nmeaToDecimal(float coordinate, char sector);
int gpsValidate(const char *nmea);
void gpsParse(GPS_Data_t *gps, char *line);
char* getField(char* line, int field_num);
float getLat(GPS_Data_t *gps);
float getLong(GPS_Data_t *gps);


#ifdef __cplusplus
}
#endif

#endif /* INC_GPS_H_ */
