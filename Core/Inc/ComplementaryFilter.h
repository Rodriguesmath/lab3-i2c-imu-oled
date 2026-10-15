/******************************************************************************/
/**
 * @file    ComplementaryFilter.h
 * @brief   Filtro complementar para fusao sensorial de atitude (Roll, Pitch, Yaw).
 * @author  Rodrigues - Formacao em Sistemas Embarcados Virtus-CC
 * @note    Estilo de codificacao inspirado nas normas do Laboratorio ASSERT (IFPB).
 * @{
 ******************************************************************************/
#ifndef _COMPLEMENTARY_FILTER_H_
#define _COMPLEMENTARY_FILTER_H_

/*******************************************************************************
 * INCLUDES NECESSARIOS
 ******************************************************************************/
#include "AssertTypes.h"
#include "Mpu6050.h"

/*******************************************************************************
 * CONFIGURACOES
 ******************************************************************************/
#define dFILTER_DEFAULT_ALPHA             0.98f

/*******************************************************************************
 * TIPOS DE DADOS PUBLICOS
 ******************************************************************************/
typedef struct
{
    float rollDeg;
    float pitchDeg;
    float yawDeg;
    float alpha;
} pose_t;

/*******************************************************************************
 * PROTOTIPOS PUBLICOS
 ******************************************************************************/
void ComplementaryFilter_Init(pose_t *pPose, float alpha);
void ComplementaryFilter_Update(pose_t *pPose, const mpu6050Data_t *pImuData, float dtS);

#endif /* _COMPLEMENTARY_FILTER_H_ */
/** @} DOXYGEN GROUP TAG END OF FILE */
