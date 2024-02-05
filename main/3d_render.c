#include "3d_render.h"

#include <math.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define LCOS_ADDRESS 0x48
#define I2C_MASTER_TIMEOUT_MS 200
#define I2C_MASTER_SCL_IO 47
#define I2C_MASTER_SDA_IO 48

static const char *TAG = "3d_render";

void lcosi2cwrite(uint8_t addr, uint8_t pcmd) // 写单个字节命令
{
    uint8_t write_buf[2] = {addr, pcmd};
    i2c_master_write_to_device(I2C_NUM_0, LCOS_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void initI2C()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // 配置 SDA 的 GPIO
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO, // 配置 SCL 的 GPIO
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000, // 为项目选择频率
        .clk_flags = 0,             // 可选项，可以使用 I2C_SCLK_SRC_FLAG_* 标志来选择 I2C 源时钟
    };

    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    ESP_LOGI(TAG, "i2c_param_config %d", ret);
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    ESP_LOGI(TAG, "i2c_driver_install %d", ret);
    ESP_LOGI(TAG, "i2c_0_master_init OK");
}

void initLcosI2CComand()
{
    lcosi2cwrite(0x00, 0b00010110); // R00h
    lcosi2cwrite(0x0A, 0xF2);
    lcosi2cwrite(0x0B, 0x80);
    lcosi2cwrite(0x0C, 0x00);
    lcosi2cwrite(0x0D, 0x51);
    lcosi2cwrite(0x0E, 0x8A);
    lcosi2cwrite(0x0F, 0xB2);
    lcosi2cwrite(0x10, 0xD8);
    lcosi2cwrite(0x11, 0xE5);
    lcosi2cwrite(0x12, 0x19);
    lcosi2cwrite(0x13, 0x26);
    lcosi2cwrite(0x14, 0x4C);
    lcosi2cwrite(0x15, 0x74);
    lcosi2cwrite(0x16, 0xAD);
    lcosi2cwrite(0x17, 0xFF);
}

// Point3 perspectiveProjection(Point3 p)
// {
//     float x1 = Camx + fac_a * p.x + fac_b * p.y + fac_c * p.z;
//     float y1 = Camy + fac_d * p.x + fac_e * p.y + fac_f * p.z;
//     // float z1 = (2 * far * near) / (-far + near) + (Camz * (far + near)) / (-far + near) + (far + near) * (-cr *cr * cy *cy * sp - cy *cy * sp * sr *sr - cr *cr * sp * sy *sy - sp * sr *sr * sy *sy) * p.x / (-far + near) + (cp * (far + near) * sr * 1.0) * p.y / (-far + near) + cp * cr * (far + near)  * p.z / (-far + near);
//     float w = -Camz + fac_g * p.x - fac_h * p.y - fac_i * p.z;
//     Point3 resultPoint;
//     resultPoint.x = (0.5 * (x1 / w + 1) * 320);
//     resultPoint.y = (0.5 * (1 - y1 / w) * 240);
//     resultPoint.z = w;
//     return resultPoint;
// }

// // 计算重心坐标
// void cacuCenterOfGravity(Point3 p0, Point3 p1, Point3 p2, float *alpha, float *beta, float *gamma,float x,float y)
// {
//     *alpha = (-(x - p1.x) * (p2.y - p1.y) + (y - p1.y) * (p2.x - p1.x)) / (-(p0.x - p1.x) * (p2.y - p1.y) + (p0.y - p1.y) * (p2.x - p1.x));
//     *beta = (-(x - p2.x) * (p0.y - p2.y) + (y - p2.y) * (p0.x - p2.x)) / (-(p1.x - p2.x) * (p0.y - p2.y) + (p1.y - p2.y) * (p0.x - p2.x));
//     *gamma = 1.0f - *alpha - *beta;
// }