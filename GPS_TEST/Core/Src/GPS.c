#include "GPSConfig.h"
#include "GPS.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

GPS_t GPS;
//##################################################################################################################
double convertDegMinToDecDeg (float degMin)
{
  double min = 0.0;
  double decDeg = 0.0;
 
  //get the minutes, fmod() requires double
  min = fmod((double)degMin, 100.0);
 
  //rebuild coordinates in decimal degrees
  degMin = (int) ( degMin / 100 );
  decDeg = degMin + ( min / 60 );
 
  return decDeg;
}
//##################################################################################################################
void	GPS_Init(void)
{
	GPS.rxIndex=0;
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);	
}
//##################################################################################################################
void	GPS_CallBack(void)
{
	GPS.LastTime=HAL_GetTick();
	if(GPS.rxIndex < sizeof(GPS.rxBuffer)-2)
	{
		GPS.rxBuffer[GPS.rxIndex] = GPS.rxTmp;
		GPS.rxIndex++;
	}	
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
}
//##################################################################################################################
void	GPS_Process(void)
{
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0))
	{
		char	*str;
		char 	*token;
		char	tmp[5];
		#if (_GPS_DEBUG==1)
		printf("%s",GPS.rxBuffer);
		#endif
		str=strstr((char*)GPS.rxBuffer,"$GPGGA,");
		if(str!=NULL){
			token = strtok(str, ",");
		   /* walk through other tokens */
		   if( token != NULL ) {
			   GPS.GPGGA.DATA_TYPE = token;
			   token = strtok(NULL, ",");

			   GPS.GPGGA.UTC_Hour = token;
			   token = strtok(NULL, ",");

//			   GPS.GPGGA.Status = token;
//			   token = strtok(NULL, ",");

			   GPS.GPGGA.Latitude = floor(strtod(token, NULL) / 100) + (strtod(token, NULL) - floor(strtod(token, NULL) / 100) * 100)/100;
			   token = strtok(NULL, ",");

			   GPS.GPGGA.NS_Indicator = token;
			   token = strtok(NULL, ",");

			   GPS.GPGGA.Longitude = strtod(token, NULL);
			   token = strtok(NULL, ",");

			   GPS.GPGGA.EW_Indicator = token;
			   token = strtok(NULL, ",");
		   }

		}		
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
}
//##################################################################################################################