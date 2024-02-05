#ifndef TD_RENDER_H
#define TD_RENDER_H
#include <stdio.h>
#include "lvgl.h"


#define Pi 3.1415926


typedef struct
{
    float x, y, z;
} Point3;


static float fac_a;
static float fac_b;
static float fac_c;
static float fac_d;
static float fac_e;
static float fac_f;
static float fac_g;
static float fac_h;
static float fac_i;
static float fac_yaw;
static float fac_pitch;
static float fac_roll;
static float Camx = 0;
static float Camy = 0;
static float Camz = 0;
// static float near = 0.1;
// static float far = 1000;






void initI2C();
void initLcosI2CComand();
void lcosi2cwrite(uint8_t addr, uint8_t pcmd);



#endif