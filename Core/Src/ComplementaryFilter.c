/******************************************************************************/
/**
 * @file ComplementaryFilter.c
 * @brief Filtro complementar de atitude em quaternion (giroscopio + gravidade).
 * @note Correcao proporcional de gravidade inspirada no filtro explicito de
 *       Mahony et al. (2008), DOI 10.1109/TAC.2008.923738, sem magnetometro.
 *       Yaw absoluto nao e observavel com acelerometro e giroscopio apenas.
 ******************************************************************************/
#include "ComplementaryFilter.h"
#include <math.h>
#include <stddef.h>

#define dDEG_TO_RAD (0.01745329251994329577f)
#define dRAD_TO_DEG (57.29577951308232088f)

static void ComplementaryFilter_Publish(pose_t *pPose)
{
    float w = pPose->q[0], x = pPose->q[1], y = pPose->q[2], z = pPose->q[3];
    float sinPitch = 2.0f * (w*y - z*x);
    sinPitch = fmaxf(-1.0f, fminf(1.0f, sinPitch));
    pPose->pitchDeg = asinf(sinPitch) * dRAD_TO_DEG;
    // Quando o eixo X aponta quase na vertical, heading/Roll de Euler sao
    // indeterminados. Nao apresentar saltos como uma direcao confiavel.
    float horizontalSquared = 1.0f - sinPitch*sinPitch;
    pPose->yawValid = horizontalSquared > (pPose->yawValid ? 0.01f : 0.0225f);
    if(pPose->yawValid)
    {
        pPose->rollDeg = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * dRAD_TO_DEG;
        pPose->yawDeg = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)) * dRAD_TO_DEG;
    }
}

void ComplementaryFilter_Init(pose_t *pPose, float alpha)
{
    if(pPose == NULL) return;
    pPose->rollDeg = pPose->pitchDeg = pPose->yawDeg = 0.0f;
    pPose->alpha = (isfinite(alpha) && alpha > 0.0f && alpha <= 1.0f)
                   ? alpha : dFILTER_DEFAULT_ALPHA;
    pPose->isAligned = false;
    pPose->yawValid = false;
    pPose->q[0] = 1.0f;
    pPose->q[1] = pPose->q[2] = pPose->q[3] = 0.0f;
    pPose->tiltMismatchS = 0.0f;
    pPose->tiltRecoveries = 0;
}

void ComplementaryFilter_Update(pose_t *pPose, const mpu6050Data_t *pImuData, float dtS)
{
    if(!pPose || !pImuData || !isfinite(dtS) || dtS < 0.0f) return;
    float ax = pImuData->accelXG, ay = pImuData->accelYG, az = pImuData->accelZG;
    float gx = pImuData->gyroXDps * dDEG_TO_RAD;
    float gy = pImuData->gyroYDps * dDEG_TO_RAD;
    float gz = pImuData->gyroZDps * dDEG_TO_RAD;
    if(!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
       !isfinite(gx) || !isfinite(gy) || !isfinite(gz)) return;
    float accelNorm = sqrtf(ax*ax + ay*ay + az*az);

    if(!pPose->isAligned)
    {
        if(accelNorm < 0.85f || accelNorm > 1.15f) return;
        float roll = atan2f(ay, az);
        float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
        float cr = cosf(roll*0.5f), sr = sinf(roll*0.5f);
        float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
        // q = Rz(0) * Ry(pitch) * Rx(roll), corpo -> mundo.
        pPose->q[0] = cr*cp; pPose->q[1] = sr*cp;
        pPose->q[2] = cr*sp; pPose->q[3] = -sr*sp;
        pPose->isAligned = true;
        ComplementaryFilter_Publish(pPose);
        return;
    }
    if(dtS == 0.0f)
    {
        pPose->tiltMismatchS = 0.0f; // Exige evidencia continua apos uma lacuna.
        return;
    }

    float w = pPose->q[0], x = pPose->q[1], y = pPose->q[2], z = pPose->q[3];
    if(accelNorm >= 0.85f && accelNorm <= 1.15f)
    {
        ax /= accelNorm; ay /= accelNorm; az /= accelNorm;
        // Gravidade prevista no referencial do sensor.
        float vx = 2.0f*(x*z - w*y);
        float vy = 2.0f*(y*z + w*x);
        float vz = 1.0f - 2.0f*(x*x + y*y);
        float dot = ax*vx + ay*vy + az*vz;
        float gyroSquared = gx*gx + gy*gy + gz*gz;
        // Em 180 graus, o produto vetorial pode zerar mesmo com atitude
        // errada (equilibrio antipodal). Recupera a inclinacao se a gravidade
        // permanecer oposta por 0,5 s, com gyro abaixo de 5 graus/s.
        if(dot < -0.95f && gyroSquared < 0.0076154355f)
            pPose->tiltMismatchS += dtS;
        else pPose->tiltMismatchS = 0.0f;
        if(pPose->tiltMismatchS >= 0.5f)
        {
            // Rotacao minima de a para v, aplicada a direita do quaternion.
            float rw = 1.0f + fmaxf(-1.0f, fminf(1.0f, dot));
            float rx = ay*vz - az*vy, ry = az*vx - ax*vz, rz = ax*vy - ay*vx;
            float rn = sqrtf(rw*rw + rx*rx + ry*ry + rz*rz);
            if(rn < 0.00001f)
            {
                // Caso exatamente oposto: privilegia uma meia volta em Roll.
                // Esta observacao nao permite recuperar Yaw perdido.
                rw = 0;
                if(fabsf(ax) < 0.9f)
                { rx = 1.0f - ax*ax; ry = -ax*ay; rz = -ax*az; }
                else
                { rx = -ay*ax; ry = 1.0f - ay*ay; rz = -ay*az; }
                rn = sqrtf(rx*rx + ry*ry + rz*rz);
            }
            rw /= rn; rx /= rn; ry /= rn; rz /= rn;
            pPose->q[0] = w*rw - x*rx - y*ry - z*rz;
            pPose->q[1] = w*rx + x*rw + y*rz - z*ry;
            pPose->q[2] = w*ry - x*rz + y*rw + z*rx;
            pPose->q[3] = w*rz + x*ry - y*rx + z*rw;
            float qn = sqrtf(pPose->q[0]*pPose->q[0] + pPose->q[1]*pPose->q[1] +
                             pPose->q[2]*pPose->q[2] + pPose->q[3]*pPose->q[3]);
            for(u8 i = 0; i < 4; i++) pPose->q[i] /= qn;
            pPose->tiltMismatchS = 0.0f;
            pPose->tiltRecoveries++;
            ComplementaryFilter_Publish(pPose);
            return;
        }
        float gain = -logf(pPose->alpha) / 0.01f;
        // Erro = gravidade medida x prevista. Sem referencia de Norte.
        gx += gain * (ay*vz - az*vy);
        gy += gain * (az*vx - ax*vz);
        gz += gain * (ax*vy - ay*vx);
    }
    else pPose->tiltMismatchS = 0.0f;
    // Incremento quaternion exato para velocidade constante nesta amostra.
    // Integra os tres eixos ACOPLADOS: omega_z isolado nao e a taxa de Yaw
    // quando o sensor esta inclinado.
    float rate = sqrtf(gx*gx + gy*gy + gz*gz);
    float halfAngle = 0.5f * rate * dtS;
    float scale = (rate > 0.000001f) ? sinf(halfAngle)/rate : 0.5f*dtS;
    float dw = cosf(halfAngle), dx = gx*scale, dy = gy*scale, dz = gz*scale;
    pPose->q[0] = w*dw - x*dx - y*dy - z*dz;
    pPose->q[1] = w*dx + x*dw + y*dz - z*dy;
    pPose->q[2] = w*dy - x*dz + y*dw + z*dx;
    pPose->q[3] = w*dz + x*dy - y*dx + z*dw;
    float norm = sqrtf(pPose->q[0]*pPose->q[0] + pPose->q[1]*pPose->q[1] +
                       pPose->q[2]*pPose->q[2] + pPose->q[3]*pPose->q[3]);
    for(u8 i = 0; i < 4; i++) pPose->q[i] /= norm;
    ComplementaryFilter_Publish(pPose);
}
