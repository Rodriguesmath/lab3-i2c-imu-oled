/******************************************************************************/
/**
 * @file    Ssd1306.h
 * @brief   Driver I2C para Display OLED 128x64 SSD1306 com animacao de pose.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/
#ifndef _SSD1306_H_
#define _SSD1306_H_

/*******************************************************************************
 * INCLUDES NECESSARIOS
 ******************************************************************************/
#include "AssertTypes.h"
#include "ComplementaryFilter.h"
#include "Mpu6050.h"

/*******************************************************************************
 * CONFIGURACOES
 ******************************************************************************/
#define dSSD1306_I2C_ADDR                 (0x3C << 1)
#define dSSD1306_WIDTH                    128
#define dSSD1306_HEIGHT                   64

/*******************************************************************************
 * PROTOTIPOS PUBLICOS
 ******************************************************************************/
bool Ssd1306_Init(void);
void Ssd1306_Clear(void);
void Ssd1306_UpdateScreen(void);
void Ssd1306_DrawPixel(u8 x, u8 y, bool state);
void Ssd1306_DrawLine(u8 x0, u8 y0, u8 x1, u8 y1, bool state);
void Ssd1306_PrintString(u8 x, u8 y, const char *pStr);
void Ssd1306_RenderDashboard(const mpu6050Data_t *pImu, const pose_t *pPose);

#endif /* _SSD1306_H_ */
/** @} DOXYGEN GROUP TAG END OF FILE */
