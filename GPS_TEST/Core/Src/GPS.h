#ifndef _GPS_H_
#define _GPS_H_

#include <stdint.h>

//##################################################################################################################

typedef struct
{
    char* Message_ID ;
    char* Time;
    char* Data_Valid ;
    char* Raw_Latitude ;
    char* N_S ;
    char* Raw_Longitude;
    char* E_W ;
    char* Speed;
    char* COG;
    char* Date;
    char* Magnetic_Variation;
    char* M_E_W;
    char* Positioning_Mode;
    float Altitude;

    char* tmp_misc;

    float Latitude;
	float Longitude;

	
}GPGGA_t;

typedef struct 
{
	uint8_t		rxBuffer[256];
	uint16_t	rxIndex;
	uint8_t		rxTmp;	
	uint32_t	LastTime;	
	
	GPGGA_t		GPGGA;
	
}GPS_t;

extern GPS_t GPS;
//##################################################################################################################
void	GPS_Init(void);
void	GPS_CallBack(void);
void	GPS_Process(void);
//##################################################################################################################

#endif
