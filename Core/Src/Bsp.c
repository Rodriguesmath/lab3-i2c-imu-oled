/******************************************************************************/
/**
 * @file    Bsp.c
 * @brief   Implementacao do BSP para Lab 3 I2C (MPU-6050 + OLED SSD1306).
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "Bsp.h"

/*******************************************************************************
 * DEFINES LOCAIS (fixos, apenas auxiliar para calculos)
 ******************************************************************************/
#define dBSP_I2C_TIMEOUT_MS               100

/*******************************************************************************
 * CONSTANTES
 ******************************************************************************/

/*******************************************************************************
 * ESTRUTURAS DE DADOS LOCAIS
 ******************************************************************************/
static struct
{
    I2C_HandleTypeDef hi2c1;
    bool isInitialized;
} bsp;

/*******************************************************************************
 * PROTOTIPOS LOCAIS
 ******************************************************************************/
static void Bsp_SystemClockConfig(void);
static void Bsp_GpioInit(void);
static void Bsp_I2cInit(void);

/*******************************************************************************
 * FUNCOES PUBLICAS
 ******************************************************************************/

void Bsp_Init(void)
{
    HAL_Init();
    Bsp_SystemClockConfig();
    Bsp_GpioInit();
    Bsp_I2cInit();

    bsp.isInitialized = true;
}

void Bsp_LedToggle(void)
{
    HAL_GPIO_TogglePin(dBSP_LED_PORT, dBSP_LED_PIN);
}

bool Bsp_I2cMemWrite(u16 devAddr, u8 memAddr, const u8 *pData, u16 length)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&bsp.hi2c1, devAddr, (uint16_t)memAddr, 
                               I2C_MEMADD_SIZE_8BIT, (uint8_t *)pData, length, dBSP_I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool Bsp_I2cMemRead(u16 devAddr, u8 memAddr, u8 *pData, u16 length)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&bsp.hi2c1, devAddr, (uint16_t)memAddr, 
                              I2C_MEMADD_SIZE_8BIT, pData, length, dBSP_I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

bool Bsp_I2cTransmit(u16 devAddr, const u8 *pData, u16 length)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&bsp.hi2c1, devAddr, (uint8_t *)pData, length, dBSP_I2C_TIMEOUT_MS);
    return (status == HAL_OK);
}

/*******************************************************************************
 * FUNCOES LOCAIS
 ******************************************************************************/

static void Bsp_SystemClockConfig(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

#if defined(STM32F411xE)
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
#endif
}

static void Bsp_GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* LED On-Board */
    HAL_GPIO_WritePin(dBSP_LED_PORT, dBSP_LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = dBSP_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dBSP_LED_PORT, &GPIO_InitStruct);
}

static void Bsp_I2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* PB6: I2C1_SCL | PB7: I2C1_SDA (Open-Drain com Pull-Up) */
    GPIO_InitStruct.Pin = dBSP_I2C_SCL_PIN | dBSP_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    bsp.hi2c1.Instance = I2C1;
#if defined(STM32H723xx)
    bsp.hi2c1.Init.Timing = 0x00901954;
#else
    bsp.hi2c1.Init.ClockSpeed = dBSP_I2C_CLOCK_SPEED_HZ;
    bsp.hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
#endif
    bsp.hi2c1.Init.OwnAddress1 = 0;
    bsp.hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    bsp.hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    bsp.hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    bsp.hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&bsp.hi2c1);
}

/** @} DOXYGEN GROUP TAG END OF FILE */
