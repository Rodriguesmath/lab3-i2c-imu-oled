/* USER CODE BEGIN Header */
/******************************************************************************/
/**
 * @file    main.c
 * @brief   Estimacao de Pose de Aeronave via Filtro Complementar no MPU-6050
 *          com exibicao e animacao no Display OLED SSD1306.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 ******************************************************************************/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Bsp.h"
#include "ComplementaryFilter.h"
#include "Mpu6050.h"
#include "Ssd1306.h"

/* Periodos independentes: aquisicao prioritária, OLED por paginas. */
#define dSAMPLE_INTERVAL_MS       10U
#define dDISPLAY_INTERVAL_MS      100U
#define dOLED_BUDGET_MS           6U
#define dPAGE_INTERVAL_MS         2000U

typedef struct
{
    u32 signature;
    u32 samples;
    u32 imuErrors;
    u32 oledErrors;
    u32 frames;
    u32 missedDeadlines;
    u32 maxSampleIntervalMs;
    u32 calibrationAttempts;
} appDiagnostics_t;

/* Disponivel ao debugger para verificar o firmware e a temporizacao. */
volatile appDiagnostics_t appDiagnostics = {.signature = 0x49324334UL};
static mpu6050Data_t imuData;
static pose_t aircraftPose;

int main(void)
{
    Bsp_Init();
    bool oledOk = Ssd1306_Init();
    bool mpuOk = Mpu6050_Init();
    if(!mpuOk || !oledOk)
    {
        if(oledOk) (void)Ssd1306_ShowMessage("IMU INIT ERROR");
        while(1)
        {
            Bsp_LedToggle();
            HAL_Delay(100);
        }
    }

    // Nao aceita silenciosamente uma janela de calibracao com movimento.
    do
    {
        (void)Ssd1306_ShowMessage("KEEP STILL - CAL");
        appDiagnostics.calibrationAttempts++;
        mpuOk = Mpu6050_CalibrateGyro();
        if(!mpuOk)
        {
            (void)Ssd1306_ShowMessage("CAL RETRY - STILL");
            Bsp_LedToggle();
            HAL_Delay(500);
        }
    } while(!mpuOk);

    ComplementaryFilter_Init(&aircraftPose, dFILTER_DEFAULT_ALPHA);
    u32 nextSample = HAL_GetTick();
    u32 nextDisplay = nextSample;
    u32 pageEpoch = nextSample;
    u32 lastSample = nextSample;
    u32 lastHeartbeat = nextSample;
    bool previousValid = false;
    bool dataValid = false;

    while(1)
    {
        u32 now = HAL_GetTick();
        if((s32)(now - nextSample) >= 0)
        {
            // Timestamp do inicio de cada leitura; aritmetica suporta wrap do tick.
            u32 sampleTime = now;
            if(Mpu6050_ReadSensor(&imuData))
            {
                u32 interval = sampleTime - lastSample;
                if(previousValid && interval > appDiagnostics.maxSampleIntervalMs)
                    appDiagnostics.maxSampleIntervalMs = interval;
                float dt = previousValid ? (float)interval * 0.001f : 0.0f;
                // Nao integra uma lacuna longa como se o movimento fosse conhecido.
                if(interval > 100U) dt = 0.0f;
                ComplementaryFilter_Update(&aircraftPose, &imuData, dt);
                lastSample = sampleTime;
                previousValid = true;
                dataValid = true;
                appDiagnostics.samples++;
            }
            else
            {
                previousValid = false;
                dataValid = false;
                appDiagnostics.imuErrors++;
            }
            nextSample += dSAMPLE_INTERVAL_MS;
            now = HAL_GetTick();
            if((s32)(now - nextSample) >= 0)
            {
                appDiagnostics.missedDeadlines++;
                nextSample = now + dSAMPLE_INTERVAL_MS;
            }
        }

        now = HAL_GetTick();
        if((s32)(now - nextDisplay) >= 0 && !Ssd1306_IsBusy() &&
           (s32)(nextSample - now) >= (s32)dOLED_BUDGET_MS)
        {
            bool telemetry = (((now - pageEpoch) / dPAGE_INTERVAL_MS) & 1U) != 0;
            if(!Ssd1306_RenderDashboard(&imuData, &aircraftPose, telemetry, dataValid))
            {
                oledOk = false;
                appDiagnostics.oledErrors++;
            }
            nextDisplay += dDISPLAY_INTERVAL_MS;
            if((s32)(now - nextDisplay) >= 0) nextDisplay = now + dDISPLAY_INTERVAL_MS;
        }

        now = HAL_GetTick();
        if(Ssd1306_IsBusy() && (s32)(nextSample - now) >= (s32)dOLED_BUDGET_MS)
        {
            if(!Ssd1306_UpdateScreenStep())
            {
                oledOk = false;
                appDiagnostics.oledErrors++;
            }
            else if(!Ssd1306_IsBusy())
            {
                oledOk = true;
                appDiagnostics.frames++;
            }
        }

        now = HAL_GetTick();
        if(now - lastHeartbeat >= ((dataValid && oledOk) ? 500U : 100U))
        {
            Bsp_LedToggle();
            lastHeartbeat = now;
        }
        HAL_Delay(1);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while(1) {}
}
