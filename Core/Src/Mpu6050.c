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

    // 1. Verifica identidade do sensor (esperado: 0x68)
    if((Mpu6050_ReadRegisters(dMPU6050_REG_WHO_AM_I, &whoAmI, 1) == false) || (whoAmI != 0x68))
    {
        mpu6050.isConnected = false;
        return false;
    }

    // 2. Acorda o dispositivo (PWR_MGMT_1: 0x01 = Clock PLL Giroscopio X)
    Mpu6050_WriteRegister(dMPU6050_REG_PWR_MGMT_1, 0x01);

    // 3. Taxa de amostragem interna: 100 Hz (SMPLRT_DIV = 9)
    Mpu6050_WriteRegister(dMPU6050_REG_SMPLRT_DIV, 0x09);

    // 4. Filtro Passa-Baixas Digital (DLPF): 44 Hz
    Mpu6050_WriteRegister(dMPU6050_REG_CONFIG, 0x03);

    // 5. Giroscopio +/- 250 graus/s
    Mpu6050_WriteRegister(dMPU6050_REG_GYRO_CONFIG, 0x00);

    // 6. Acelerometro +/- 2g
    Mpu6050_WriteRegister(dMPU6050_REG_ACCEL_CONFIG, 0x00);

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

    pData->gyroXDps = (float)pData->rawGyroX / dGYRO_SCALE_FACTOR_250;
    pData->gyroYDps = (float)pData->rawGyroY / dGYRO_SCALE_FACTOR_250;
    pData->gyroZDps = (float)pData->rawGyroZ / dGYRO_SCALE_FACTOR_250;

    pData->tempC     = ((float)pData->rawTemp / dTEMP_DIVISOR) + dTEMP_OFFSET;

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
