#include "MPU6050.h"
#include "i2c.h"
#include <math.h>

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

float Ax, Ay, Az, Gx, Gy, Gz;

static float degrees_pitch_acc, degrees_roll_acc;
static float acc_vector;

float degrees_pitch = 0;
float degrees_roll = 0;
float temp;

void MPU_Process(void){
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
