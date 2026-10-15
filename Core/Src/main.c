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

/* Private defines -----------------------------------------------------------*/
#define dSAMPLE_INTERVAL_MS       10    // 100 Hz de taxa de amostragem
#define dDISPLAY_DECIMATION       10    // Atualiza a tela a cada 10 amostras (10 Hz)
#define dDT_SECONDS               0.01f // 100 Hz -> dt = 0.01s

/* Private variables ---------------------------------------------------------*/
static mpu6050Data_t imuData;
static pose_t        aircraftPose;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* Inicializacao do Hardware de baixo nivel */
    Bsp_Init();

    /* USER CODE BEGIN 2 */
    // Inicializa MPU-6050 e Display OLED SSD1306 no barramento I2C
    bool mpuOk = Mpu6050_Init();
    bool oledOk = Ssd1306_Init();

    if((mpuOk == false) || (oledOk == false))
    {
        // Se houver falha de comunicacao, pisca rapidamente o LED
        while(1)
        {
            Bsp_LedToggle();
            HAL_Delay(100);
        }
    }

    // Inicializa o Filtro Complementar com fator alfa = 0.98
    ComplementaryFilter_Init(&aircraftPose, dFILTER_DEFAULT_ALPHA);

    u8 displayDecimationCounter = 0;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while(1)
    {
        u32 loopStartTime = HAL_GetTick();

        // 1. Leitura dos 14 registradores do MPU-6050 em modo burst
        if(Mpu6050_ReadSensor(&imuData) == true)
        {
            // 2. Atualizacao da pose (Roll, Pitch, Yaw) via filtro complementar
            ComplementaryFilter_Update(&aircraftPose, &imuData, dDT_SECONDS);
        }

        // 3. Atualizacao do Dashboard no OLED a cada 100 ms (10 Hz)
        displayDecimationCounter++;
        if(displayDecimationCounter >= dDISPLAY_DECIMATION)
        {
            displayDecimationCounter = 0;
            Ssd1306_RenderDashboard(&imuData, &aircraftPose);
            Bsp_LedToggle();
        }

        // Aguarda proximo periodo de 10 ms (100 Hz)
        u32 elapsed = HAL_GetTick() - loopStartTime;
        if(elapsed < dSAMPLE_INTERVAL_MS)
        {
            HAL_Delay(dSAMPLE_INTERVAL_MS - elapsed);
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

void Error_Handler(void)
{
    __disable_irq();
    while(1)
    {
    }
}
