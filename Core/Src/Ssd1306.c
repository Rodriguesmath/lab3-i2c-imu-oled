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
    bool updatePending;
    u8 nextPage;
} ssd1306;

/*******************************************************************************
 * PROTOTIPOS LOCAIS
 ******************************************************************************/
static bool Ssd1306_WriteCommand(u8 cmd);

/*******************************************************************************
 * FUNCOES PUBLICAS
 ******************************************************************************/

bool Ssd1306_Init(void)
{
    // Modo PAGINA: comandos B0..B7 e endereco de coluna usados no envio.
    static const u8 commands[] = {
        0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4,
        0xD3, 0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12,
        0xDB, 0x20, 0x8D, 0x14, 0xAF
    };
    ssd1306.isReady = false;
    ssd1306.updatePending = false;
    for(u16 i = 0; i < sizeof(commands); i++)
    {
        if(!Ssd1306_WriteCommand(commands[i])) return false;
    }
    ssd1306.isReady = true;
    Ssd1306_Clear();
    if(!Ssd1306_UpdateScreen())
    {
        ssd1306.isReady = false;
        return false;
    }
    return true;
}

void Ssd1306_Clear(void)
{
    memset(ssd1306.buffer, 0, dSSD1306_BUFFER_SIZE);
}

bool Ssd1306_IsBusy(void)
{
    return ssd1306.updatePending;
}

bool Ssd1306_UpdateScreenStep(void)
{
    if(!ssd1306.isReady) return false;
    if(!ssd1306.updatePending) return true;
    u8 page = ssd1306.nextPage;
    if(!Ssd1306_WriteCommand(0xB0 + page) ||
       !Ssd1306_WriteCommand(0x00) || !Ssd1306_WriteCommand(0x10) ||
       !Bsp_I2cMemWrite(dSSD1306_I2C_ADDR, dDATA_MODE,
                        &ssd1306.buffer[dSSD1306_WIDTH * page], dSSD1306_WIDTH))
    {
        ssd1306.updatePending = false;
        return false;
    }
    if(++ssd1306.nextPage == 8) ssd1306.updatePending = false;
    return true;
}

bool Ssd1306_UpdateScreen(void)
{
    if(!ssd1306.isReady) return false;
    ssd1306.nextPage = 0;
    ssd1306.updatePending = true;
    while(ssd1306.updatePending)
    {
        if(!Ssd1306_UpdateScreenStep()) return false;
    }
    return true;
}

bool Ssd1306_ShowMessage(const char *pMessage)
{
    if(!ssd1306.isReady || ssd1306.updatePending) return false;
    Ssd1306_Clear();
    Ssd1306_PrintString(0, 20, pMessage);
    return Ssd1306_UpdateScreen();
}

void Ssd1306_DrawPixel(s16 x, s16 y, bool state)
{
    if((x < 0) || (y < 0) || (x >= dSSD1306_WIDTH) || (y >= dSSD1306_HEIGHT))
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

void Ssd1306_DrawLine(s16 x0, s16 y0, s16 x1, s16 y1, bool state)
{
    s32 dx = abs(x1 - x0);
    s16 sx = x0 < x1 ? 1 : -1;
    s32 dy = -abs(y1 - y0);
    s16 sy = y0 < y1 ? 1 : -1;
    s32 err = dx + dy;
    s32 e2;

    while(1)
    {
        Ssd1306_DrawPixel(x0, y0, state);
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
        if((c >= 'a') && (c <= 'z'))
        {
            c -= ('a' - 'A');
        }

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

static void Ssd1306_Rect(s16 x0, s16 y0, s16 x1, s16 y1, bool fill, bool state)
{
    if(fill)
    {
        for(s16 y = y0; y <= y1; y++)
            for(s16 x = x0; x <= x1; x++) Ssd1306_DrawPixel(x, y, state);
    }
    else
    {
        Ssd1306_DrawLine(x0, y0, x1, y0, state);
        Ssd1306_DrawLine(x0, y1, x1, y1, state);
        Ssd1306_DrawLine(x0, y0, x0, y1, state);
        Ssd1306_DrawLine(x1, y0, x1, y1, state);
    }
}

static void Ssd1306_DrawHeading(const pose_t *pose)
{
    char text[8];
    Ssd1306_DrawLine(0, 53, 127, 53, true);
    if(pose->yawValid)
    {
        // Escala circular: 359 -> 000 sem salto de coordenadas.
        float heading = fmodf(pose->yawDeg + 360.0f, 360.0f);
        int base = (int)floorf(heading / 10.0f) * 10;
        for(int mark = base - 70; mark <= base + 70; mark += 10)
        {
            int x = 64 + (int)lroundf((float)mark - heading);
            if(x < 0 || x > 127 || (x >= 43 && x <= 85)) continue;
            Ssd1306_DrawLine(x, 54, x, (mark % 30 == 0) ? 56 : 55, true);
            if(mark % 30 == 0 && x >= 9 && x <= 118)
            {
                int degrees = (mark % 360 + 360) % 360;
                snprintf(text, sizeof(text), "%03d", degrees);
                Ssd1306_PrintString((u8)(x - 8), 57, text);
            }
        }
        int degrees = ((int)lroundf(heading)) % 360;
        snprintf(text, sizeof(text), "%03dR", degrees);
    }
    else snprintf(text, sizeof(text), "---R");
    Ssd1306_Rect(44, 54, 83, 63, true, false);
    Ssd1306_Rect(44, 54, 83, 63, false, true);
    Ssd1306_PrintString(52, 56, text);
    Ssd1306_DrawLine(62, 51, 64, 53, true);
    Ssd1306_DrawLine(66, 51, 64, 53, true);
}

static void Ssd1306_DrawAttitude(const pose_t *pose, bool gyroPage)
{
    float w = pose->q[0], x = pose->q[1], y = pose->q[2], z = pose->q[3];
    float gx = 2.0f*(x*z - w*y);
    float gy = 2.0f*(y*z + w*x);
    float gz = 1.0f - 2.0f*(x*x + y*y);
    float edge = 0.6f * sqrtf(gy*gy + gz*gz);
    // Plano do horizonte projetado diretamente da gravidade do quaternion.
    // Nao usa tan(pitch), Euler congelado ou clamp de extremos da reta.
    // h>0 indica solo; invertido, o solo passa continuamente para cima.
    for(s16 row = 11; row <= 50; row++)
    {
        float h = gz * (row - 30) + gy * (34 - 64) + 20.0f * gx;
        for(s16 col = 34; col <= 94; col++, h += gy)
        {
            bool ground = h > 0.0f && ((col + row) & 3) == 0;
            bool horizon = edge > 0.0001f && fabsf(h) <= edge;
            Ssd1306_DrawPixel(col, row, ground || horizon);
        }
    }
    Ssd1306_DrawLine(32, 11, 32, 51, true);
    Ssd1306_DrawLine(96, 11, 96, 51, true);
    // Marcas fixas de banco em -60, -30, 0, 30, 60 graus.
    for(int angle = -60; angle <= 60; angle += 30)
    {
        float rad = angle * 0.01745329252f;
        s16 x0 = 64 + (s16)lroundf(17*sinf(rad));
        s16 y0 = 30 - (s16)lroundf(17*cosf(rad));
        s16 x1 = 64 + (s16)lroundf(19*sinf(rad));
        s16 y1 = 30 - (s16)lroundf(19*cosf(rad));
        Ssd1306_DrawLine(x0, y0, x1, y1, true);
    }
    // Aeronave fixa com fundo limpo, legivel sobre ceu e solo pontilhado.
    Ssd1306_Rect(53, 28, 75, 34, true, false);
    Ssd1306_DrawLine(55, 30, 61, 30, true);
    Ssd1306_DrawLine(67, 30, 73, 30, true);
    Ssd1306_DrawLine(61, 30, 64, 33, true);
    Ssd1306_DrawLine(64, 33, 67, 30, true);
    const char *status = !pose->yawValid ? "VERT" : (gz < -0.15f) ? "INV" : NULL;
    if(status)
    {
        Ssd1306_Rect(50, 18, 78, 25, true, false);
        Ssd1306_PrintString(52, 18, status);
    }
    Ssd1306_Rect(44, 43, 86, 50, true, false);
    Ssd1306_PrintString(50, 43, gyroPage ? "W D/S" : "A G");
}

bool Ssd1306_RenderDashboard(const mpu6050Data_t *pImu, const pose_t *pPose,
                             bool telemetryPage, bool dataValid)
{
    if(!ssd1306.isReady || ssd1306.updatePending || !pImu || !pPose) return false;
    Ssd1306_Clear();
    if(!dataValid)
    {
        Ssd1306_PrintString(20, 18, "IMU ERROR");
        Ssd1306_PrintString(20, 32, "DATA STALE");
    }
    else if(!pPose->isAligned)
    {
        Ssd1306_PrintString(20, 24, "ATT WAIT");
    }
    else
    {
        char text[32];
        // Faixa superior alterna os seis eixos sem retirar o painel de voo.
        if(telemetryPage)
            snprintf(text, sizeof(text), "WX%+04.0f WY%+04.0f WZ%+04.0f",
                     (double)pImu->gyroXDps, (double)pImu->gyroYDps, (double)pImu->gyroZDps);
        else
            snprintf(text, sizeof(text), "AX%+.1f AY%+.1f AZ%+.1f",
                     (double)pImu->accelXG, (double)pImu->accelYG, (double)pImu->accelZG);
        Ssd1306_PrintString(0, 0, text);
        Ssd1306_DrawLine(0, 9, 127, 9, true);
        Ssd1306_DrawAttitude(pPose, telemetryPage);
        Ssd1306_PrintString(0, 12, "ROLL");
        if(pPose->yawValid) snprintf(text, sizeof(text), "%+04.0f", (double)pPose->rollDeg);
        else snprintf(text, sizeof(text), " -- ");
        Ssd1306_PrintString(0, 22, text);
        Ssd1306_PrintString(103, 12, "PIT");
        snprintf(text, sizeof(text), "%+03.0f", (double)pPose->pitchDeg);
        Ssd1306_PrintString(103, 22, text);
        Ssd1306_PrintString(0, 35, "NZ");
        snprintf(text, sizeof(text), "%+.1f", (double)pImu->accelZG);
        Ssd1306_PrintString(0, 45, text);
        Ssd1306_PrintString(103, 35, "WZ");
        snprintf(text, sizeof(text), "%+04.0f", (double)pImu->gyroZDps);
        Ssd1306_PrintString(103, 45, text);
        Ssd1306_DrawHeading(pPose);
    }
    ssd1306.nextPage = 0;
    ssd1306.updatePending = true;
    return true;
}

static bool Ssd1306_WriteCommand(u8 cmd)
{
    return Bsp_I2cMemWrite(dSSD1306_I2C_ADDR, dCMD_MODE, &cmd, 1);
}

/** @} DOXYGEN GROUP TAG END OF FILE */
