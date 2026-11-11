#include "Bsp.h"
#include "Mpu6050.h"
#include "ComplementaryFilter.h"
#include "Ssd1306.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static u32 tick;
static u8 registers[128];
static bool readFail, writeFail, moving;
static int failedRegister = -1;
static int oledFailAt = -1;
static unsigned oledWrites, pageWrites, burstReads;
static u8 currentPage, screen[1024];

void HAL_Delay(uint32_t ms) { tick += ms; }
uint32_t HAL_GetTick(void) { return tick; }

bool Bsp_I2cMemWrite(u16 address, u8 reg, const u8 *data, u16 length)
{
    if(writeFail) return false;
    if(address == dMPU6050_I2C_ADDR)
    {
        if(reg == failedRegister) return false;
        assert(length == 1);
        registers[reg] = *data;
        return true;
    }
    assert(address == dSSD1306_I2C_ADDR);
    if((int)oledWrites++ == oledFailAt) return false;
    if(reg == 0 && length == 1 && (*data & 0xF8) == 0xB0) currentPage = *data & 7;
    if(reg == 0x40)
    {
        assert(length == 128);
        memcpy(&screen[currentPage * 128], data, 128);
        pageWrites++;
    }
    return true;
}

bool Bsp_I2cMemRead(u16 address, u8 reg, u8 *data, u16 length)
{
    assert(address == dMPU6050_I2C_ADDR);
    if(readFail) return false;
    assert((unsigned)reg + length <= sizeof(registers));
    memcpy(data, &registers[reg], length);
    if(reg == 0x3B)
    {
        assert(length == 14);
        burstReads++;
        if(moving)
        {
            s16 rate = (burstReads & 1U) ? 393 : -131;
            data[8] = (u8)((u16)rate >> 8);
            data[9] = (u8)rate;
        }
    }
    return true;
}

static void raw(u8 reg, s16 value)
{
    registers[reg] = (u8)((u16)value >> 8);
    registers[reg + 1] = (u8)value;
}

static void test_sensor(void)
{
    mpu6050Data_t data;
    registers[0x75] = 0x68;
    readFail = true;
    assert(!Mpu6050_Init());
    assert(!Mpu6050_ReadSensor(&data));
    readFail = false;
    const u8 config[] = {0x6B, 0x19, 0x1A, 0x1B, 0x1C};
    for(unsigned i = 0; i < sizeof(config); i++)
    {
        failedRegister = config[i];
        assert(!Mpu6050_Init());
        assert(!Mpu6050_ReadSensor(&data));
    }
    failedRegister = -1;
    assert(Mpu6050_Init());
    assert(registers[0x19] == 9 && registers[0x1A] == 3);
    raw(0x3B, -16384); raw(0x3D, 8192); raw(0x3F, 16384);
    raw(0x41, 340); raw(0x43, 131); raw(0x45, -262); raw(0x47, 393);
    assert(Mpu6050_ReadSensor(&data));
    assert(data.accelXG == -1 && data.accelYG == 0.5f && data.accelZG == 1);
    assert(data.gyroXDps == 1 && data.gyroYDps == -2 && data.gyroZDps == 3);
    assert(fabsf(data.tempC - 37.53f) < 0.001f);
    raw(0x3B, 0); raw(0x3D, 0);
    u32 before = tick;
    assert(Mpu6050_CalibrateGyro());
    assert(tick - before == 2000);
    assert(Mpu6050_ReadSensor(&data));
    assert(data.gyroXDps == 0 && data.gyroYDps == 0 && data.gyroZDps == 0);
    assert(data.rawGyroX == 131); // Valores brutos preservados.
    moving = true;
    assert(!Mpu6050_CalibrateGyro());
    moving = false;
    assert(Mpu6050_ReadSensor(&data) && data.gyroXDps == 0); // Bias anterior preservado.
    readFail = true;
    assert(!Mpu6050_ReadSensor(&data));
    assert(!Mpu6050_CalibrateGyro());
    readFail = false;
}

static float angle_error(float actual, float expected)
{
    return fabsf(remainderf(actual - expected, 360.0f));
}

static void test_filter(void)
{
    pose_t pose;
    mpu6050Data_t data = {.accelYG = 1};
    ComplementaryFilter_Init(&pose, 0.98f);
    ComplementaryFilter_Update(&pose, &data, 0);
    assert(angle_error(pose.rollDeg, 90) < 0.001f);

    // Com Roll=90, um giro em torno do Y DO SENSOR corresponde a Yaw
    // do mundo. Integrar apenas gyroZ falhava neste caso.
    data.gyroYDps = 90;
    for(unsigned i = 0; i < 100; i++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(angle_error(pose.yawDeg, 90) < 0.02f);
    assert(angle_error(pose.rollDeg, 90) < 0.02f);
    assert(fabsf(pose.pitchDeg) < 0.02f);
    data.gyroYDps = -90;
    for(unsigned i = 0; i < 100; i++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(angle_error(pose.yawDeg, 0) < 0.02f);

    // Intervalos variaveis: 50*(5+15) ms = 1 s, com 50 graus/s.
    ComplementaryFilter_Init(&pose, 0.98f);
    data = (mpu6050Data_t){.accelZG = 1};
    ComplementaryFilter_Update(&pose, &data, 0);
    data.gyroZDps = 50;
    for(unsigned i = 0; i < 50; i++)
    {
        ComplementaryFilter_Update(&pose, &data, 0.005f);
        ComplementaryFilter_Update(&pose, &data, 0.015f);
    }
    assert(angle_error(pose.yawDeg, 50) < 0.01f);
    pose_t before = pose;
    ComplementaryFilter_Update(&pose, &data, 0);
    ComplementaryFilter_Update(&pose, &data, NAN);
    ComplementaryFilter_Update(&pose, &data, -1);
    assert(memcmp(&pose, &before, sizeof(pose)) == 0);

    // Convergencia da referencia gravitacional (sinal da realimentacao).
    ComplementaryFilter_Init(&pose, 0.98f);
    data = (mpu6050Data_t){.accelZG = 1};
    ComplementaryFilter_Update(&pose, &data, 0);
    data.accelYG = 0.5f; data.accelZG = sqrtf(0.75f);
    for(unsigned i = 0; i < 500; i++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(angle_error(pose.rollDeg, 30) < 0.01f);

    // Aceleracao externa de 2g nao deve ser interpretada como gravidade.
    ComplementaryFilter_Init(&pose, 0.98f);
    data = (mpu6050Data_t){.accelZG = 1};
    ComplementaryFilter_Update(&pose, &data, 0);
    data.accelXG = 2; data.accelZG = 0;
    ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(fabsf(pose.pitchDeg) < 0.001f);

    // Cruza +/-180 sem interpolar pelo zero.
    ComplementaryFilter_Init(&pose, 0.98f);
    data = (mpu6050Data_t){.accelYG = 0.0174524f, .accelZG = -0.9998477f};
    ComplementaryFilter_Update(&pose, &data, 0);
    data.accelYG = -0.0174524f;
    ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(fabsf(pose.rollDeg) > 179);

    // Pitch 90: quaternion permanece valido e a UI nao inventa um heading.
    ComplementaryFilter_Init(&pose, 1.0f);
    data = (mpu6050Data_t){.accelZG = 1};
    ComplementaryFilter_Update(&pose, &data, 0);
    data.gyroYDps = 90;
    for(unsigned i = 0; i < 100; i++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(!pose.yawValid && fabsf(pose.pitchDeg - 90) < 0.05f);
    for(unsigned i = 0; i < 4; i++) assert(isfinite(pose.q[i]));
    data.gyroYDps = -90;
    for(unsigned i = 0; i < 100; i++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(pose.yawValid && fabsf(pose.pitchDeg) < 0.02f);

    // Vinte trajetorias fechadas em tres eixos. A atitude final e identidade:
    // Rx(+90), Ry_corpo(+90), Rx_corpo(-90), Rz_corpo(-90).
    // No integrador antigo, cada ciclo acumulava -90 graus de Yaw.
    ComplementaryFilter_Init(&pose, 0.98f);
    data = (mpu6050Data_t){.accelZG = 1};
    ComplementaryFilter_Update(&pose, &data, 0);
    for(unsigned cycle = 0; cycle < 20; cycle++)
    {
        for(unsigned phase = 0; phase < 4; phase++)
        {
            for(unsigned n = 0; n < 100; n++)
            {
                float roll = (phase == 0) ? n * 0.9f :
                             (phase == 1) ? 90.0f : (phase == 2) ? 90.0f - n * 0.9f : 0.0f;
                float rad = roll * 0.01745329252f;
                data = (mpu6050Data_t){.accelYG = sinf(rad), .accelZG = cosf(rad)};
                if(phase == 0) data.gyroXDps = 90;
                if(phase == 1) data.gyroYDps = 90;
                if(phase == 2) data.gyroXDps = -90;
                if(phase == 3) data.gyroZDps = -90;
                ComplementaryFilter_Update(&pose, &data, 0.01f);
            }
        }
        assert(angle_error(pose.yawDeg, 0) < 0.1f);
        assert(angle_error(pose.rollDeg, 0) < 0.1f);
        assert(fabsf(pose.pitchDeg) < 0.1f);
    }
    float norm = 0;
    for(unsigned i = 0; i < 4; i++) norm += pose.q[i]*pose.q[i];
    assert(fabsf(norm - 1) < 0.00001f);
}

static void test_inversions(void)
{
    pose_t pose;
    mpu6050Data_t data = {.accelZG = -1};
    ComplementaryFilter_Init(&pose, 0.98f);
    ComplementaryFilter_Update(&pose, &data, 0);
    assert(pose.isAligned && fabsf(pose.rollDeg) > 179);
    assert(pose.tiltRecoveries == 0); // Inicializacao ja invertida.

    ComplementaryFilter_Init(&pose, 0.98f);
    data.accelZG = 1;
    ComplementaryFilter_Update(&pose, &data, 0);
    data.accelZG = -1; // Inversao cujo giro nao foi adquirido.
    for(unsigned n = 0; n < 40; n++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    ComplementaryFilter_Update(&pose, &data, 0); // Falha interrompe a janela.
    for(unsigned n = 0; n < 40; n++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(pose.tiltRecoveries == 0);
    for(unsigned n = 0; n < 70; n++) ComplementaryFilter_Update(&pose, &data, 0.01f);
    assert(pose.tiltRecoveries == 1);
    assert(fabsf(pose.rollDeg) > 179 && fabsf(pose.pitchDeg) < 0.01f);
    assert(isfinite(pose.yawDeg));

    for(unsigned axis = 0; axis < 2; axis++)
    {
        ComplementaryFilter_Init(&pose, 0.98f);
        data = (mpu6050Data_t){.accelZG = 1};
        ComplementaryFilter_Update(&pose, &data, 0);
        for(unsigned n = 0; n < 400; n++)
        {
            float rad = n * 0.9f * 0.01745329252f;
            data = (mpu6050Data_t){.accelZG = cosf(rad)};
            if(axis == 0) { data.accelYG = sinf(rad); data.gyroXDps = 90; }
            else { data.accelXG = -sinf(rad); data.gyroYDps = 90; }
            float previous[4]; memcpy(previous, pose.q, sizeof(previous));
            ComplementaryFilter_Update(&pose, &data, 0.01f);
            float continuity = 0, norm = 0;
            for(unsigned i = 0; i < 4; i++)
            {
                assert(isfinite(pose.q[i]));
                continuity += previous[i] * pose.q[i];
                norm += pose.q[i] * pose.q[i];
            }
            assert(continuity > 0.99f && fabsf(norm-1) < 0.00001f);
        }
        assert(pose.tiltRecoveries == 0); // Giro acompanhado nao causa reset.
        assert(pose.yawValid && angle_error(pose.rollDeg, 0) < 0.1f);
        assert(fabsf(pose.pitchDeg) < 0.1f && angle_error(pose.yawDeg, 0) < 0.1f);
    }
}

static pose_t panel_pose(float roll, float pitch, float yaw)
{
    pose_t pose = {.rollDeg=roll, .pitchDeg=pitch, .yawDeg=yaw,
                   .isAligned=true, .yawValid=fabsf(cosf(pitch*0.01745329252f)) > 0.15f};
    float r=roll*0.00872664626f, p=pitch*0.00872664626f, y=yaw*0.00872664626f;
    pose.q[0]=cosf(r)*cosf(p)*cosf(y)+sinf(r)*sinf(p)*sinf(y);
    pose.q[1]=sinf(r)*cosf(p)*cosf(y)-cosf(r)*sinf(p)*sinf(y);
    pose.q[2]=cosf(r)*sinf(p)*cosf(y)+sinf(r)*cosf(p)*sinf(y);
    pose.q[3]=cosf(r)*cosf(p)*sinf(y)-sinf(r)*sinf(p)*cosf(y);
    return pose;
}

static void drain(void)
{
    unsigned before = pageWrites;
    for(unsigned i = 0; i < 8; i++)
    {
        assert(Ssd1306_IsBusy());
        assert(Ssd1306_UpdateScreenStep());
        assert(pageWrites == before + i + 1);
    }
    assert(!Ssd1306_IsBusy());
}

static void test_display(void)
{
    mpu6050Data_t data = {.accelZG = 1, .gyroXDps = 10, .gyroYDps = -20, .gyroZDps = 30};
    pose_t pose = panel_pose(0, 0, 0);
    writeFail = true;
    assert(!Ssd1306_Init());
    assert(!Ssd1306_UpdateScreenStep());
    writeFail = false;
    assert(Ssd1306_Init());
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true));
    assert(!Ssd1306_RenderDashboard(&data, &pose, false, true));
    drain();
    u8 initial[1024]; memcpy(initial, screen, sizeof(screen));
    // Ambos os eixos antes ausentes alteram a telemetria renderizada.
    data.gyroXDps = 11;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain();
    assert(memcmp(initial, screen, sizeof(screen)) != 0);
    memcpy(initial, screen, sizeof(screen)); data.gyroYDps = -21;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain();
    assert(memcmp(initial, screen, sizeof(screen)) != 0);
    memcpy(initial, screen, sizeof(screen)); pose.yawDeg = 90;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain();
    bool leftChanged = false;
    for(unsigned page = 0; page < 8; page++)
        if(memcmp(initial + page*128, screen + page*128, 48)) leftChanged = true;
    assert(leftChanged); // Yaw altera o grafico, alem do texto.
    pose.yawValid = false;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain();
    memcpy(initial, screen, sizeof(screen));
    pose.yawDeg += 45;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain();
    assert(memcmp(initial, screen, sizeof(screen)) == 0); // Heading invalido fica oculto, sem direcao inventada.
    pose.yawValid = true;
    assert(Ssd1306_RenderDashboard(&data, &pose, false, false)); drain();
    memcpy(initial, screen, sizeof(screen));
    data.gyroXDps = 100;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, false)); drain();
    assert(memcmp(initial, screen, sizeof(screen)) == 0); // Tela de erro nao exibe valores antigos.
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true));
    oledFailAt = (int)oledWrites + 3; // Falha no payload da pagina.
    assert(!Ssd1306_UpdateScreenStep());
    assert(!Ssd1306_IsBusy());
    oledFailAt = -1;
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain();
    // Exercita limites e todos os quadrantes das animacoes (sanitizers opcionais).
    for(int angle = -180; angle <= 180; angle += 15)
    {
        pose = panel_pose(angle, angle / 2.0f, angle);
        assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
    }
}

static bool pixel(unsigned x, unsigned y)
{
    return (screen[x + (y/8)*128] & (1U<<(y%8))) != 0;
}

static void save_frame(const char *name)
{
    char path[128]; snprintf(path, sizeof(path), "build/panel-%s.bin", name);
    FILE *file = fopen(path, "wb"); assert(file);
    assert(fwrite(screen, 1, sizeof(screen), file) == sizeof(screen));
    assert(fclose(file) == 0);
}

static void test_panel(void)
{
    mpu6050Data_t data = {.accelZG=1, .gyroXDps=10, .gyroYDps=-20, .gyroZDps=30};
    const float angles[][3] = {{0,0,0},{35,15,40},{180,0,0},{0,90,0},{0,-90,0},{0,0,359},{0,0,0},{0,0,1}};
    const char *names[] = {"level","bank","inverted","up","down","359","000","001"};
    for(unsigned n = 0; n < 8; n++)
    {
        pose_t pose = panel_pose(angles[n][0], angles[n][1], angles[n][2]);
        data.accelXG = 2*(pose.q[1]*pose.q[3] - pose.q[0]*pose.q[2]);
        data.accelYG = 2*(pose.q[2]*pose.q[3] + pose.q[0]*pose.q[1]);
        data.accelZG = 1-2*(pose.q[1]*pose.q[1] + pose.q[2]*pose.q[2]);
        assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
        save_frame(names[n]);
        if(n == 0) assert(!pixel(40,20) && pixel(40,40));
        if(n == 2) assert(pixel(40,20) && !pixel(40,40)); // Solo acima, invertido.
        if(n == 3) assert(!pixel(40,20) && !pixel(40,40)); // Ceu, nariz acima.
        if(n == 4) assert(pixel(40,20) && pixel(40,40)); // Solo, nariz abaixo.
        u8 expected[1024]; memcpy(expected, screen, sizeof(screen));
        for(unsigned i = 0; i < 4; i++) pose.q[i] = -pose.q[i];
        assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
        assert(memcmp(expected, screen, sizeof(screen)) == 0); // q e -q: mesmo desenho.
    }
    pose_t pose = panel_pose(0,0,0);
    // A fita representa a mesma direcao em ambos os lados de 0/360 e +/-180.
    const float equivalent[][2] = {{-1,359},{0,360},{1,361},{-180,180}};
    for(unsigned n = 0; n < 4; n++)
    {
        pose = panel_pose(0,0,equivalent[n][0]);
        assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
        u8 expected[1024]; memcpy(expected, screen, sizeof(screen));
        pose = panel_pose(0,0,equivalent[n][1]);
        assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
        assert(memcmp(expected, screen, sizeof(screen)) == 0);
    }
    pose = panel_pose(0,0,0);
    assert(Ssd1306_RenderDashboard(&data, &pose, true, true)); drain(); save_frame("rates");
    u8 before[1024]; memcpy(before, screen, sizeof(screen));
    data.accelXG = 0.5f;
    assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
    memcpy(before, screen, sizeof(screen)); data.accelXG = 0.6f;
    assert(Ssd1306_RenderDashboard(&data, &pose, false, true)); drain();
    assert(memcmp(before, screen, sizeof(screen)) != 0);
    // Primitivas recortam coordenadas negativas, sem wrap de u8.
    Ssd1306_Clear();
    Ssd1306_DrawPixel(-1,-1,true);
    Ssd1306_DrawLine(-5,20,5,20,true);
    assert(Ssd1306_UpdateScreen());
    assert(pixel(0,20) && pixel(5,20) && !pixel(127,20));
}

int main(void)
{
    test_sensor(); test_filter(); test_inversions(); test_display(); test_panel();
    puts("PASS: sensores, 20 trajetorias, inversoes 360/antipodal, painel/clipping/q=-q, OLED");
    return 0;
}
