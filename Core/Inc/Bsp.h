/******************************************************************************/
/**
 * @file    Bsp.h
 * @brief   Board Support Package para Lab 3 I2C (MPU-6050 + OLED SSD1306).
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/
#ifndef _BSP_H_
#define _BSP_H_

/*******************************************************************************
 * INCLUDES NECESSARIOS
 ******************************************************************************/
#include "AssertTypes.h"

#if defined(STM32F411xE) || defined(STM32F401xC)
  #include "stm32f4xx_hal.h"
#elif defined(STM32H723xx)
  #include "stm32h7xx_hal.h"
#elif defined(STM32F767xx)
  #include "stm32f7xx_hal.h"
#else
  #include "stm32f4xx_hal.h"
#endif

/*******************************************************************************
 * CONFIGURACOES
 ******************************************************************************/
#define dBSP_I2C_CLOCK_SPEED_HZ           400000UL // 400 kHz Fast Mode

/*******************************************************************************
 * DEFINES PUBLICOS
 ******************************************************************************/
#define dBSP_I2C_SCL_PIN                  GPIO_PIN_6
#define dBSP_I2C_SCL_PORT                 GPIOB
#define dBSP_I2C_SDA_PIN                  GPIO_PIN_7
#define dBSP_I2C_SDA_PORT                 GPIOB

#if defined(STM32F411xE)
  #define dBSP_LED_PIN                    GPIO_PIN_13
  #define dBSP_LED_PORT                   GPIOC
#elif defined(STM32F767xx)
  #define dBSP_LED_PIN                    GPIO_PIN_0
  #define dBSP_LED_PORT                   GPIOB
#else
  #define dBSP_LED_PIN                    GPIO_PIN_13
  #define dBSP_LED_PORT                   GPIOC
#endif

/*******************************************************************************
 * PROTOTIPOS PUBLICOS
 ******************************************************************************/
void Bsp_Init(void);
void Bsp_LedToggle(void);
bool Bsp_I2cMemWrite(u16 devAddr, u8 memAddr, const u8 *pData, u16 length);
bool Bsp_I2cMemRead(u16 devAddr, u8 memAddr, u8 *pData, u16 length);
bool Bsp_I2cTransmit(u16 devAddr, const u8 *pData, u16 length);

#endif /* _BSP_H_ */
/** @} DOXYGEN GROUP TAG END OF FILE */
