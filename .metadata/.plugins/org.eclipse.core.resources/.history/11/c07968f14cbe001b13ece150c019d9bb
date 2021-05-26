/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "GPS.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// BMP180 PTD
void InitBMP(void);
uint16_t Get_UTemp (void);
float BMP180_GetTemp (void);
uint32_t Get_UPress (int oss);
float BMP180_GetPress (int oss);
float BMP180_GetAlt (int oss);

// MPU6050 PTD
void MPU6050_Read_Gyro (void);
void MPU6050_Read_Accel ();
void MPU6050_Init (void);
void Calibration_MPU6050(float data);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP180_I2C &hi2c1
#define BMP180_ADDRESS 0xEE
#define atmPress 101325 //Pa

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
short AC1 = 0;
short AC2 = 0;
short AC3 = 0;
unsigned short AC4 = 0;
unsigned short AC5 = 0;
unsigned short AC6 = 0;
short B1 = 0;
short B2 = 0;
short MB = 0;
short MC = 0;
short MD = 0;
long UT = 0;
short oss = 0;
long UP = 0;
long X1 = 0;
long X2 = 0;
long X3 = 0;
long B3 = 0;
long B5 = 0;
unsigned long B4 = 0;
long B6 = 0;
unsigned long B7 = 0;
long Press = 0;
long Temp = 0;
float Temperature = 0;
float Pressure = 0;
float Altitude = 0;

float Ax, Ay, Az, Gx, Gy, Gz;

static float degrees_pitch_acc, degrees_roll_acc;
static float acc_vector;
static float degrees_pitch = 0, degrees_roll = 0;
float temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  GPS_CallBack();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  GPS_Init();
  InitBMP();
  MPU6050_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	GPS_Process();


	Temperature = BMP180_GetTemp();
	Pressure = BMP180_GetPress(0);
	Altitude = BMP180_GetAlt(0);


	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();

	degrees_pitch += Gy * 0.0000610687;
	degrees_roll  += Gx * 0.0000610687;

	degrees_pitch += degrees_roll * sin(Gz * 0.000001066);
	degrees_roll  -= degrees_pitch * sin(Gz * 0.000001066);

	acc_vector = sqrt((Ax * Ax) + (Ay * Ay) + (Az * Az));
	degrees_pitch_acc = asin((float) Ay/acc_vector) * 57.2957795;
	degrees_roll_acc  = asin((float) Ax/acc_vector) * -57.2957795;

	degrees_pitch = degrees_pitch * 0.97 + degrees_pitch_acc * 0.03;
	degrees_roll  = degrees_roll * 0.97 + degrees_roll_acc * 0.03;

	if(degrees_pitch > 20.0 || degrees_roll > 20.0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void InitBMP(void){
	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, Callib_Start, 1, Callib_Data,22, HAL_MAX_DELAY);

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);
}

uint16_t Get_UTemp (void)
{
	uint8_t datatowrite = 0x2E;
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay (5);  // wait 4.5 ms
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Temp_RAW, 2, 1000);
	return ((Temp_RAW[0]<<8) + Temp_RAW[1]);
}

float BMP180_GetTemp (void)
{
	UT = Get_UTemp();
	X1 = ((UT-AC6) * (AC5/(pow(2,15))));
	X2 = ((MC*(pow(2,11))) / (X1+MD));
	B5 = X1+X2;
	Temp = (B5+8)/(pow(2,4));
	return Temp/10.0;
}

uint32_t Get_UPress (int oss)   // over sampling settings 0,1,2,3
{
	uint8_t datatowrite = 0x34+(oss<<6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_ADDRESS, 0xF4, 1, &datatowrite, 1, 1000);
	switch (oss)
	{
		case (0):
			HAL_Delay (5);
			break;
		case (1):
			HAL_Delay (8);
			break;
		case (2):
			HAL_Delay (14);
			break;
		case (3):
			HAL_Delay (26);
			break;
	}
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_ADDRESS, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0]<<16)+(Press_RAW[1]<<8)+Press_RAW[2]) >> (8-oss));
}

float BMP180_GetPress (int oss)
{
	UP = Get_UPress(oss);

	B6 = B5 - 4000;

	X1 = (B2*(B6*B6/pow(2,12)))/pow(2,11);
	X2 = AC2 * B6 / pow(2,11);
	X3 = X1 + X2;

	B3 = (((AC1*4+X3) << oss) + 2) / 4;

	X1 = AC3 * B6 / pow(2,13);
	X2 = (B1 * (B6 * B6 / pow(2,12))) / pow(2,16);
	X3 = ((X1+X2)+2) / pow(2,2);

	B4 = AC4 * (unsigned long)(X3 + 32768) / pow(2,15);
	B7 = ((unsigned long)UP - B3)* (50000 >> oss);

	if(B7 < 0x80000000)
		Press = (B7 * 2) / B4;
	else
		Press = (B7 / B4) * 2;

	X1 = (Press / pow(2,8)) * (Press / pow(2,8));
	X1 = (X1 * 3038) / pow (2,16);
	X2 = (-7357 * Press) / pow(2,16);

	Press = Press + (X1 + X2 + 3791) / pow(2,4);

	return Press;
}



float BMP180_GetAlt (int oss)
{
	BMP180_GetPress (oss);
	return 44330*(1-(pow((Press/(float)atmPress), 0.19029495718)));
}


void MPU6050_Init (void)
{
	  uint8_t PWR_MGMT_1[2] = {0x6B, 0x00};
	  while (HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, PWR_MGMT_1, 2, 10) != HAL_OK);

	  uint8_t GYR_CONFIG[2] = {0x1B, 0x08};
	  while(HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, GYR_CONFIG, 2, 10) != HAL_OK);

	  uint8_t ACC_CONFIG[2] = {0x1C, 0x10};
	  while(HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, ACC_CONFIG, 2, 10) != HAL_OK);

	  uint8_t LPF_CONFIG[2] = {0x1A, 0x03};
	  while(HAL_I2C_Master_Transmit(&hi2c2, MPU6050_ADDR, LPF_CONFIG, 2, 10) != HAL_OK);

}

void MPU6050_Read_Accel ()
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 100);

	Ax = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Ay = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Az = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
}

void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gx = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gy = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gz = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
