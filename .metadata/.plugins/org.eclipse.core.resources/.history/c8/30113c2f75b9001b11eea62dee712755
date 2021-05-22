/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */

#define	GPS_USART	&huart2
#define GPSBUFSIZE  128       // GPS buffer size

extern float enlem;
extern float boylam;

void GPS_Init();
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
