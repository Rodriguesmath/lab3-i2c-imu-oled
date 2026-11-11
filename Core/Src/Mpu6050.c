/******************************************************************************/
/**
 * @file    Mpu6050.c
 * @brief   Implementacao do driver do sensor MPU-6050.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "Mpu6050.h"
#include "Bsp.h"
#include <math.h>

/*******************************************************************************
 * DEFINES LOCAIS (fixos, apenas auxiliar para calculos)
 ******************************************************************************/
#define dACCEL_SCALE_FACTOR_2G            16384.0f
#define dGYRO_SCALE_FACTOR_250            131.0f
#define dTEMP_DIVISOR                     340.0f
#define dTEMP_OFFSET                      36.53f
#define dBURST_READ_LENGTH                14

/*******************************************************************************
 * CONSTANTES
 ******************************************************************************/

/*******************************************************************************
 * ESTRUTURAS DE DADOS LOCAIS
 ******************************************************************************/
static struct
{
    bool isConnected;
    float gyroBias[3];
} mpu6050;

/*******************************************************************************
 * PROTOTIPOS LOCAIS
 ******************************************************************************/
static bool Mpu6050_WriteRegister(u8 reg, u8 val);
static bool Mpu6050_ReadRegisters(u8 reg, u8 *pBuffer, u16 len);

/*******************************************************************************
 * FUNCOES PUBLICAS
 ******************************************************************************/

bool Mpu6050_Init(void)
{
    u8 whoAmI = 0;
    mpu6050.isConnected = false;
    for(u8 i = 0; i < 3; i++) mpu6050.gyroBias[i] = 0.0f;

    // 1. Verifica identidade do sensor (esperado: 0x68)
    if((Mpu6050_ReadRegisters(dMPU6050_REG_WHO_AM_I, &whoAmI, 1) == false) || (whoAmI != 0x68))
    {
        mpu6050.isConnected = false;
        return false;
    }

    // 2. Acorda o dispositivo (PWR_MGMT_1: 0x01 = Clock PLL Giroscopio X)
    if(!Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x01)) return false;
    HAL_Delay(100); // Aguarda estabilizacao do clock e sensores.

    // 3. Taxa de amostragem interna: 100 Hz (SMPLRT_DIV = 9)
    if(!Mpu6050_WriteRegister(dMPU6050_REG_SMPLRT_DIV, 0x09)) return false;

    // 4. DLPF: acelerometro 44 Hz; giroscopio 42 Hz.
    if(!Mpu6050_WriteRegister(dMPU6050_REG_CONFIG, 0x03)) return false;

    // 5. Giroscopio +/- 250 graus/s
    if(!Mpu6050_WriteRegister(dMPU6050_REG_GYRO_CONFIG, 0x00)) return false;

    // 6. Acelerometro +/- 2g
    if(!Mpu6050_WriteRegister(dMPU6050_REG_ACCEL_CONFIG, 0x00)) return false;

    mpu6050.isConnected = true;
    return true;
}

bool Mpu6050_ReadSensor(mpu6050Data_t *pData)
{
    if((pData == NULL) || (mpu6050.isConnected == false))
    {
        return false;
    }

    u8 rawBuffer[dBURST_READ_LENGTH];

    // Leitura em burst de todos os 14 registradores continuos
    if(Mpu6050_ReadRegisters(dMPU6050_REG_ACCEL_XOUT_H, rawBuffer, dBURST_READ_LENGTH) == false)
    {
        return false;
    }

    // Conversao de Big-Endian para s16 nativo
    pData->rawAccelX = (s16)((rawBuffer[0] << 8) | rawBuffer[1]);
    pData->rawAccelY = (s16)((rawBuffer[2] << 8) | rawBuffer[3]);
    pData->rawAccelZ = (s16)((rawBuffer[4] << 8) | rawBuffer[5]);
    pData->rawTemp    = (s16)((rawBuffer[6] << 8) | rawBuffer[7]);
    pData->rawGyroX  = (s16)((rawBuffer[8] << 8) | rawBuffer[9]);
    pData->rawGyroY  = (s16)((rawBuffer[10] << 8) | rawBuffer[11]);
    pData->rawGyroZ  = (s16)((rawBuffer[12] << 8) | rawBuffer[13]);

    // Conversao para unidades de engenharia (g, graus/s, graus Celsius)
    pData->accelXG  = (float)pData->rawAccelX / dACCEL_SCALE_FACTOR_2G;
    pData->accelYG  = (float)pData->rawAccelY / dACCEL_SCALE_FACTOR_2G;
    pData->accelZG  = (float)pData->rawAccelZ / dACCEL_SCALE_FACTOR_2G;

    pData->gyroXDps = (float)pData->rawGyroX / dGYRO_SCALE_FACTOR_250 - mpu6050.gyroBias[0];
    pData->gyroYDps = (float)pData->rawGyroY / dGYRO_SCALE_FACTOR_250 - mpu6050.gyroBias[1];
    pData->gyroZDps = (float)pData->rawGyroZ / dGYRO_SCALE_FACTOR_250 - mpu6050.gyroBias[2];

    pData->tempC     = ((float)pData->rawTemp / dTEMP_DIVISOR) + dTEMP_OFFSET;

    return true;
}

bool Mpu6050_CalibrateGyro(void)
{
    const u16 samples = 200;
    float sum[3] = {0}, sumSquared[3] = {0};
    float accelSum[3] = {0}, accelSquared[3] = {0};
    mpu6050Data_t data;

    for(u16 n = 0; n < samples; n++)
    {
        HAL_Delay(10); // Amostras distintas, ODR configurada em 100 Hz.
        if(!Mpu6050_ReadSensor(&data)) return false;
        float a[3] = {data.accelXG, data.accelYG, data.accelZG};
        float g[3] = {data.gyroXDps + mpu6050.gyroBias[0],
                      data.gyroYDps + mpu6050.gyroBias[1],
                      data.gyroZDps + mpu6050.gyroBias[2]};
        float normSquared = a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
        if((normSquared < 0.81f) || (normSquared > 1.21f)) return false;
        for(u8 i = 0; i < 3; i++)
        {
            if(fabsf(g[i]) > 10.0f) return false;
            sum[i] += g[i];
            sumSquared[i] += g[i] * g[i];
            accelSum[i] += a[i];
            accelSquared[i] += a[i] * a[i];
        }
    }
    for(u8 i = 0; i < 3; i++)
    {
        float mean = sum[i] / samples;
        float accelMean = accelSum[i] / samples;
        if((sumSquared[i] / samples - mean * mean > 0.25f) ||
           (accelSquared[i] / samples - accelMean * accelMean > 0.0004f)) return false;
    }
    // Publica os tres offsets somente depois de validar a janela inteira.
    for(u8 i = 0; i < 3; i++) mpu6050.gyroBias[i] = sum[i] / samples;
    return true;
}

void Mpu6050_Sleep(void)
{
    Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x40);
}

void Mpu6050_Wakeup(void)
{
    Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x01);
}

/*******************************************************************************
 * FUNCOES LOCAIS
 ******************************************************************************/

static bool Mpu6050_WriteRegister(u8 reg, u8 val)
{
    return Bsp_I2cMemWrite(dMPU6050_I2C_ADDR, reg, &val, 1);
}

static bool Mpu6050_ReadRegisters(u8 reg, u8 *pBuffer, u16 len)
{
    return Bsp_I2cMemRead(dMPU6050_I2C_ADDR, reg, pBuffer, len);
}

/** @} DOXYGEN GROUP TAG END OF FILE */
