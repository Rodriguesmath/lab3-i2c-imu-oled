/******************************************************************************/
/**
 * @file    Ssd1306.c
 * @brief   Implementacao do driver do Display OLED SSD1306 128x64.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "Ssd1306.h"
#include "Bsp.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * DEFINES LOCAIS (fixos, apenas auxiliar para calculos)
 ******************************************************************************/
#define dSSD1306_BUFFER_SIZE              (dSSD1306_WIDTH * dSSD1306_HEIGHT / 8)
#define dCMD_MODE                         0x00
#define dDATA_MODE                        0x40

/*******************************************************************************
 * CONSTANTES
 ******************************************************************************/
// Fonte ASCII 5x7 simplificada
static const u8 font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Espaco
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}  // Z
};

/*******************************************************************************
 * ESTRUTURAS DE DADOS LOCAIS
 ******************************************************************************/
static struct
{
    u8   buffer[dSSD1306_BUFFER_SIZE];
    bool isReady;
} ssd1306;

/*******************************************************************************
 * PROTOTIPOS LOCAIS
 ******************************************************************************/
static void Ssd1306_WriteCommand(u8 cmd);

/*******************************************************************************
 * FUNCOES PUBLICAS
 ******************************************************************************/

bool Ssd1306_Init(void)
{
    // Sequencia padrao de inicializacao do display OLED SSD1306 128x64
    Ssd1306_WriteCommand(0xAE); // Display OFF
    Ssd1306_WriteCommand(0x20); // Set Memory Addressing Mode
    Ssd1306_WriteCommand(0x00); // Horizontal Addressing Mode
    Ssd1306_WriteCommand(0xB0); // Set Page Start Address
    Ssd1306_WriteCommand(0xC8); // Set COM Output Scan Direction
    Ssd1306_WriteCommand(0x00); // Set low column address
    Ssd1306_WriteCommand(0x10); // Set high column address
    Ssd1306_WriteCommand(0x40); // Set start line address
    Ssd1306_WriteCommand(0x81); // Set contrast control
    Ssd1306_WriteCommand(0xFF);
    Ssd1306_WriteCommand(0xA1); // Set segment re-map
    Ssd1306_WriteCommand(0xA6); // Normal display
    Ssd1306_WriteCommand(0xA8); // Set multiplex ratio(1 to 64)
    Ssd1306_WriteCommand(0x3F); // 1/64 duty
    Ssd1306_WriteCommand(0xA4); // Output follows RAM content
    Ssd1306_WriteCommand(0xD3); // Set display offset
    Ssd1306_WriteCommand(0x00);
    Ssd1306_WriteCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    Ssd1306_WriteCommand(0xF0);
    Ssd1306_WriteCommand(0xD9); // Set pre-charge period
    Ssd1306_WriteCommand(0x22);
    Ssd1306_WriteCommand(0xDA); // Set com pins hardware configuration
    Ssd1306_WriteCommand(0x12);
    Ssd1306_WriteCommand(0xDB); // Set vcomh
    Ssd1306_WriteCommand(0x20);
    Ssd1306_WriteCommand(0x8D); // Set DC-DC enable
    Ssd1306_WriteCommand(0x14);
    Ssd1306_WriteCommand(0xAF); // Display ON

    Ssd1306_Clear();
    Ssd1306_UpdateScreen();
    ssd1306.isReady = true;

    return true;
}

void Ssd1306_Clear(void)
{
    memset(ssd1306.buffer, 0, dSSD1306_BUFFER_SIZE);
}

void Ssd1306_UpdateScreen(void)
{
    for(u8 m = 0; m < 8; m++)
    {
        Ssd1306_WriteCommand(0xB0 + m);
        Ssd1306_WriteCommand(0x00);
        Ssd1306_WriteCommand(0x10);
        Bsp_I2cMemWrite(dSSD1306_I2C_ADDR, dDATA_MODE, &ssd1306.buffer[dSSD1306_WIDTH * m], dSSD1306_WIDTH);
    }
}

void Ssd1306_DrawPixel(u8 x, u8 y, bool state)
{
    if((x >= dSSD1306_WIDTH) || (y >= dSSD1306_HEIGHT))
    {
        return;
    }

    if(state == true)
    {
        ssd1306.buffer[x + (y / 8) * dSSD1306_WIDTH] |= (1 << (y % 8));
    }
    else
    {
        ssd1306.buffer[x + (y / 8) * dSSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void Ssd1306_DrawLine(u8 x0, u8 y0, u8 x1, u8 y1, bool state)
{
    s16 dx = abs(x1 - x0);
    s16 sx = x0 < x1 ? 1 : -1;
    s16 dy = -abs(y1 - y0);
    s16 sy = y0 < y1 ? 1 : -1;
    s16 err = dx + dy;
    s16 e2;

    while(1)
    {
        Ssd1306_DrawPixel((u8)x0, (u8)y0, state);
        if((x0 == x1) && (y0 == y1))
        {
            break;
        }
        e2 = 2 * err;
        if(e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if(e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

void Ssd1306_PrintString(u8 x, u8 y, const char *pStr)
{
    if(pStr == NULL) return;

    while(*pStr)
    {
        char c = *pStr++;
        if((c >= ' ') && (c <= 'Z'))
        {
            u8 index = c - ' ';
            for(u8 i = 0; i < 5; i++)
            {
                u8 line = font5x7[index][i];
                for(u8 j = 0; j < 8; j++)
                {
                    bool pixel = ((line >> j) & 0x01) ? true : false;
                    Ssd1306_DrawPixel(x + i, y + j, pixel);
                }
            }
        }
        x += 6;
    }
}

void Ssd1306_RenderDashboard(const mpu6050Data_t *pImu, const pose_t *pPose)
{
    Ssd1306_Clear();

    // 1. ANIMACAO DE POSE (Horizonte Artificial no lado esquerdo do display)
    // Circulo delimitador do indicador de atitude
    u8 cx = 24;
    u8 cy = 32;
    u8 radius = 18;

    // Calcula inclinacao da linha do horizonte a partir do Roll e Pitch
    float rad = (pPose->rollDeg * 3.14159f) / 180.0f;
    float pitchOffset = pPose->pitchDeg * 0.4f;

    s16 x0 = cx - (s16)(radius * cosf(rad));
    s16 y0 = cy - (s16)(radius * sinf(rad)) + (s16)pitchOffset;
    s16 x1 = cx + (s16)(radius * cosf(rad));
    s16 y1 = cy + (s16)(radius * sinf(rad)) + (s16)pitchOffset;

    // Limita coordenadas na tela
    if(y0 < 2)  y0 = 2;
    if(y0 > 62) y0 = 62;
    if(y1 < 2)  y1 = 2;
    if(y1 > 62) y1 = 62;

    Ssd1306_DrawLine((u8)x0, (u8)y0, (u8)x1, (u8)y1, true);

    // Marcador central fixo da aeronave
    Ssd1306_DrawLine(cx - 4, cy, cx + 4, cy, true);
    Ssd1306_DrawPixel(cx, cy - 2, true);

    // Divisoria vertical
    Ssd1306_DrawLine(48, 0, 48, 63, true);

    // 2. EXIBICAO NUMERICA DE ENGENHARIA (Lado direito)
    char str[20];

    // Angulos do Filtro Complementar
    snprintf(str, sizeof(str), "R:%4.0f", pPose->rollDeg);
    Ssd1306_PrintString(52, 2, str);

    snprintf(str, sizeof(str), "P:%4.0f", pPose->pitchDeg);
    Ssd1306_PrintString(52, 12, str);

    snprintf(str, sizeof(str), "Y:%4.0f", pPose->yawDeg);
    Ssd1306_PrintString(52, 22, str);

    // Aceleracao Linear XYZ (g)
    snprintf(str, sizeof(str), "A:%+.1f,%+.1f", pImu->accelXG, pImu->accelYG);
    Ssd1306_PrintString(52, 34, str);

    snprintf(str, sizeof(str), "Z:%+.1f g", pImu->accelZG);
    Ssd1306_PrintString(52, 44, str);

    // Giroscopio (graus/s)
    snprintf(str, sizeof(str), "W:%4.0f", pImu->gyroZDps);
    Ssd1306_PrintString(52, 54, str);

    Ssd1306_UpdateScreen();
}

/*******************************************************************************
 * FUNCOES LOCAIS
 ******************************************************************************/

static void Ssd1306_WriteCommand(u8 cmd)
{
    Bsp_I2cMemWrite(dSSD1306_I2C_ADDR, dCMD_MODE, &cmd, 1);
}

/** @} DOXYGEN GROUP TAG END OF FILE */
