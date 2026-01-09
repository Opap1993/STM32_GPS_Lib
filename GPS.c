/*
 * GPS.c
 *
 *  Created on: Jan 9, 2026
 *      Author: John
 */
#include "GPS.h"

static uint8_t rxBuffer[DMA_BUF_SIZE];
static volatile uint16_t rxIndex = 0;
static volatile uint8_t nmeaReady = 0;
static char nmeaLine[DMA_BUF_SIZE];

float nmeaToDecimal(float coordinate, char sector) {
    int deg = (int)(coordinate / 100.0f);
    float minutes = coordinate - (deg * 100.0f);
    float decimal = (float)deg + (minutes / 60.0f);
    if (sector == 'S' || sector == 'W') decimal = -decimal;
    return decimal;
}

int gpsValidate(const char *nmea) {
    if (!nmea || nmea[0] != '$') return 0;
    uint8_t calc = 0;
    int i = 1;
    while (nmea[i] != '*' && nmea[i] != '\0') {
        calc ^= nmea[i++];
    }
    if (nmea[i] == '*') {
        uint8_t expected = (uint8_t)strtol(&nmea[i + 1], NULL, 16);
        return (calc == expected);
    }
    return 0;
}

char* getField(char* line, int field_num) {
    static char buffer[32]; // Shared static buffer
    int current_field = 0;
    char *start = line, *end;
    while (current_field < field_num) {
        start = strchr(start, ',');
        if (!start) return NULL;
        start++;
        current_field++;
    }
    end = strchr(start, ',');
    if (!end) end = strchr(start, '*');
    if (!end) return NULL;
    int len = end - start;
    if (len <= 0 || len >= 32) return NULL;
    memcpy(buffer, start, len);
    buffer[len] = '\0';
    return buffer;
}

void gpsParse(GPS_Data_t *gps, char *line) {
    if (!line) return;
    char laS[20]={0}, loS[20]={0}, nsS[2]={0}, ewS[2]={0}, fixS[2]={0};
    char *t;

    if (strstr(line, "GLL") || strstr(line, "RMC") || strstr(line, "GGA")) {
        // Example for RMC (Adapt field numbers for GGA if needed)
        if (strstr(line, "RMC")) {
            t = getField(line, 2); if(t) strcpy(fixS, t);
            if (fixS[0] == 'A') {
                t = getField(line, 3); if(t) strcpy(laS, t);
                t = getField(line, 4); if(t) nsS[0] = t[0];
                t = getField(line, 5); if(t) strcpy(loS, t);
                t = getField(line, 6); if(t) ewS[0] = t[0];

                gps->decimalLat = nmeaToDecimal(atof(laS), nsS[0]);
                gps->decimalLong = nmeaToDecimal(atof(loS), ewS[0]);
                gps->gpsHasFix = 1;
                gps->gpsUpdated = 1;
            } else {
            	gps->gpsHasFix = 0;
            }
        }

        if (strstr(line, "GGA")) {
           	char* fixField = getField(line, 6);
           	if (fixField && atoi(fixField) > 0) {
           		char laStr[32], loStr[32], nsStr[2], ewStr[2];
           		char* temp = getField(line, 2);
           		if(temp) strcpy(laStr, temp); else return;

           		                // Get N/S and copy it
           	   temp = getField(line, 3);
           	   if(temp) strcpy(nsStr, temp); else return;

           		                // Get Longitude and copy it
           	   temp = getField(line, 4);
           	   if(temp) strcpy(loStr, temp); else return;

           		                // Get E/W and copy it
           	   temp = getField(line, 5);
           	   if(temp) strcpy(ewStr, temp); else return;

           	gps->decimalLat = nmeaToDecimal(atof(laStr), nsStr[0]);
           	gps->decimalLong = nmeaToDecimal(atof(loStr), ewStr[0]);

           	gps->gpsHasFix = 1;
           	gps->gpsUpdated = 1;
           	}
           	else {
           		gps->gpsHasFix = 0;
           	}
           }
    }
}

void GPS_Init(GPS_Data_t *gps, UART_HandleTypeDef *huart) {
    gps->huart = huart;
    gps->gpsHasFix = 0;
    gps->gpsUpdated = 0;

    // Start DMA
    HAL_UARTEx_ReceiveToIdle_DMA(gps->huart, rxBuffer, DMA_BUF_SIZE);
    // Disable Half-Transfer
    __HAL_DMA_DISABLE_IT(gps->huart->hdmarx, DMA_IT_HT);
}

void GPS_ProcessCallback(GPS_Data_t *gps, uint16_t Size) {
    // Size is the number of bytes received since the last IDLE or half/full transfer
    if (Size < DMA_BUF_SIZE) {
        // 1. Copy the raw DMA buffer into our safe nmeaLine string
        memcpy(nmeaLine, rxBuffer, Size);
        nmeaLine[Size] = '\0';

        // 2. Set the global size (optional, but good for debugging)
        rxIndex = Size;
        nmeaReady = 1;

        // 3. Restart DMA listening using the stored handle
        HAL_UARTEx_ReceiveToIdle_DMA(gps->huart, rxBuffer, DMA_BUF_SIZE);

        // 4. Disable HT using the handle pointer (Generic version of &hdma_usart2_rx)
        if (gps->huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(gps->huart->hdmarx, DMA_IT_HT);
        }
    }


    /*                          !main.c syntax!
     void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Check if the interrupt came from the GPS UART
    if (huart->Instance == USART2)
    {
        // Pass the work to the library
        GPS_ProcessCallback(&myGPS, Size);
    }
}
     */
}

void  GPS_Process(GPS_Data_t *gps) {
	gps->gpsUpdated = 0;
	if (nmeaReady) {
		 nmeaReady = 0;
		 char *ptr = nmeaLine;
		 while ((ptr = strstr(ptr, "$")) != NULL) {
		 // Look for the end of the NMEA sentence
		 char *end = strchr(ptr, '\r');
		 if (!end) end = strchr(ptr, '\n');

		 if (end != NULL) {
		     char tempChar = *end; // Save char
		     *end = '\0';          // Terminate for validator

		     if (gpsValidate(ptr)) {
		         gpsParse(gps,ptr);
		         }

		      *end = tempChar;      // Restore char
		      ptr = end + 1;        // Move to next possible sentence
		      } else {
		               break;
		              }
		          }
	}
}

float getLat(GPS_Data_t *gps){
	if(gps->gpsUpdated){
		return gps->decimalLat;
	}
	else {
		return 0;
	}
}

float getLong(GPS_Data_t *gps){
	if(gps->gpsUpdated){
		return gps->decimalLong;
	}
	else {
		return 0;
	}
}



