#include "GPSConfig.h"
#include "GPS.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

GPS_t GPS;

float GpsToDecimalDegrees(const char* nmeaPos, char quadrant){
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant=='W' || quadrant=='S')
      v= -v;
  }
  return v;
}

void	GPS_Init(void)
{
	GPS.rxIndex=0;
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);	
}

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

char *strtok_fr (char *s, char delim, char **save_ptr)
{
    char *tail;
    char c;

    if (s == NULL) {
        s = *save_ptr;
    }
    tail = s;
    if ((c = *tail) == '\0') {
        s = NULL;
    }
    else {
        do {
            if (c == delim) {
                *tail++ = '\0';
                break;
           }
        }while ((c = *++tail) != '\0');
    }
    *save_ptr = tail;
    return s;
}

char *strtok_f (char *s, char delim)
{
    static char *save_ptr;

    return strtok_fr (s, delim, &save_ptr);
}

void GPS_Process(void) //$GPGGA,031956,2218.2035,N,11410.7595,E,1,04,3,9,005.9,M,-001.3,M,,*51
{
	if( (HAL_GetTick()-GPS.LastTime>50) && (GPS.rxIndex>0)){
		char *str;
		str=strstr((char*)GPS.rxBuffer,"$GPGGA,");

		if(str!=NULL){
		    GPS.GPGGA.Message_ID = strtok_f(str,',');
		    GPS.GPGGA.Time = strtok_f(NULL,',');
//		    GPS.GPGGA.Data_Valid = strtok_f(NULL,',');
		    GPS.GPGGA.Raw_Latitude = strtok_f(NULL,',');
		    GPS.GPGGA.N_S = strtok_f(NULL,',');
		    GPS.GPGGA.Raw_Longitude = strtok_f(NULL,',');
		    GPS.GPGGA.E_W = strtok_f(NULL,',');

		    GPS.GPGGA.tmp_misc = strtok_f(NULL,',');
		    GPS.GPGGA.tmp_misc = strtok_f(NULL,',');
		    GPS.GPGGA.tmp_misc = strtok_f(NULL,',');

		    GPS.GPGGA.Altitude = atof(strtok_f(NULL,','));

		    GPS.GPGGA.Latitude = GpsToDecimalDegrees(GPS.GPGGA.Raw_Latitude, GPS.GPGGA.N_S);
		    GPS.GPGGA.Longitude = GpsToDecimalDegrees(GPS.GPGGA.Raw_Longitude, GPS.GPGGA.E_W);
		}		
		memset(GPS.rxBuffer,0,sizeof(GPS.rxBuffer));
		GPS.rxIndex=0;
	}
	HAL_UART_Receive_IT(&_GPS_USART,&GPS.rxTmp,1);
}
