/******************************************************************************/
/**
 * @file    Mpu6050.h
 * @brief   Driver para o Sensor IMU de 6 Graus de Liberdade MPU-6050.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/
#ifndef _MPU6050_H_
#define _MPU6050_H_

/*******************************************************************************
 * INCLUDES NECESSARIOS
 ******************************************************************************/
#include "AssertTypes.h"

/*******************************************************************************
 * CONFIGURACOES
 ******************************************************************************/
#define dMPU6050_I2C_ADDR                 (0x68 << 1)

/*******************************************************************************
 * DEFINES PUBLICOS
 ******************************************************************************/
#define dMPU6050_REG_SMPLRT_DIV           0x19
#define dMPU6050_REG_CONFIG               0x1A
#define dMPU6050_REG_GYRO_CONFIG          0x1B
#define dMPU6050_REG_ACCEL_CONFIG         0x1C
#define dMPU6050_REG_ACCEL_XOUT_H         0x3B
#define dMPU6050_REG_TEMP_OUT_H           0x41
#define dMPU6050_REG_GYRO_XOUT_H          0x43
#define dMPU6050_REG_PWR_MGMT_1           0x6B
#define dMPU6050_REG_WHO_AM_I             0x75

/*******************************************************************************
 * TIPOS DE DADOS PUBLICOS
 ******************************************************************************/
typedef struct
{
    float accelXG;
    float accelYG;
    float accelZG;
    float gyroXDps;
    float gyroYDps;
    float gyroZDps;
    float tempC;

    s16   rawAccelX;
    s16   rawAccelY;
    s16   rawAccelZ;
    s16   rawGyroX;
    s16   rawGyroY;
    s16   rawGyroZ;
    s16   rawTemp;
} mpu6050Data_t;

/*******************************************************************************
 * PROTOTIPOS PUBLICOS
 ******************************************************************************/
bool Mpu6050_Init(void);
bool Mpu6050_ReadSensor(mpu6050Data_t *pData);
void Mpu6050_Sleep(void);
void Mpu6050_Wakeup(void);

#endif /* _MPU6050_H_ */
/** @} DOXYGEN GROUP TAG END OF FILE */
