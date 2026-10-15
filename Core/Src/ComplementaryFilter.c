/******************************************************************************/
/**
 * @file    ComplementaryFilter.c
 * @brief   Implementacao da fusao sensorial por filtro complementar.
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "ComplementaryFilter.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define dRAD_TO_DEG                       (180.0f / (float)M_PI)

/*******************************************************************************
 * DEFINES LOCAIS (fixos, apenas auxiliar para calculos)
 ******************************************************************************/

/*******************************************************************************
 * CONSTANTES
 ******************************************************************************/

/*******************************************************************************
 * ESTRUTURAS DE DADOS LOCAIS
 ******************************************************************************/

/*******************************************************************************
 * PROTOTIPOS LOCAIS
 ******************************************************************************/

/*******************************************************************************
 * FUNCOES PUBLICAS
 ******************************************************************************/

void ComplementaryFilter_Init(pose_t *pPose, float alpha)
{
    if(pPose != NULL)
    {
        pPose->rollDeg = 0.0f;
        pPose->pitchDeg = 0.0f;
        pPose->yawDeg = 0.0f;
        pPose->alpha = alpha;
    }
}

void ComplementaryFilter_Update(pose_t *pPose, const mpu6050Data_t *pImuData, float dtS)
{
    if((pPose == NULL) || (pImuData == NULL))
    {
        return;
    }

    // 1. Calculo de Roll e Pitch baseados unicamente no acelerometro
    float rollAcc = atan2f(pImuData->accelYG, pImuData->accelZG) * dRAD_TO_DEG;
    float pitchAcc = atan2f(-pImuData->accelXG, 
                            sqrtf((pImuData->accelYG * pImuData->accelYG) + 
                                  (pImuData->accelZG * pImuData->accelZG))) * dRAD_TO_DEG;

    // 2. Filtro complementar: fusao do giroscopio (alta freq) com acelerometro (baixa freq)
    // Angle = alpha * (Angle + GyroRate * dt) + (1 - alpha) * AccAngle
    pPose->rollDeg = (pPose->alpha * (pPose->rollDeg + (pImuData->gyroXDps * dtS))) + 
                     ((1.0f - pPose->alpha) * rollAcc);

    pPose->pitchDeg = (pPose->alpha * (pPose->pitchDeg + (pImuData->gyroYDps * dtS))) + 
                      ((1.0f - pPose->alpha) * pitchAcc);

    // 3. Yaw integrado a partir do eixo Z do giroscopio
    pPose->yawDeg += (pImuData->gyroZDps * dtS);
}

/*******************************************************************************
 * FUNCOES LOCAIS
 ******************************************************************************/

/** @} DOXYGEN GROUP TAG END OF FILE */
