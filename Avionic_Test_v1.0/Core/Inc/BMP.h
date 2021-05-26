#ifndef _BMP_H_
#define _BMP_H_

#include <stdint.h>

extern float Altitude;

void BMP_Process(void);

void InitBMP(void);

uint16_t Get_UTemp (void);

float BMP180_GetTemp (void);

uint32_t Get_UPress (int oss) ;

float BMP180_GetPress (int oss);

float BMP180_GetAlt (int oss);


#endif
