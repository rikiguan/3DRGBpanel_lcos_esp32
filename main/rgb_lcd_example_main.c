#include <math.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/i2c.h"
#include "3d_render.h"

static const char *TAG = "example";

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (18 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT 38

#define EXAMPLE_PIN_NUM_HSYNC -1
#define EXAMPLE_PIN_NUM_VSYNC -1
#define EXAMPLE_PIN_NUM_DE 21
#define EXAMPLE_PIN_NUM_PCLK 14
#define EXAMPLE_PIN_NUM_DATA0 9   // B0
#define EXAMPLE_PIN_NUM_DATA1 10  // B1
#define EXAMPLE_PIN_NUM_DATA2 11  // B2
#define EXAMPLE_PIN_NUM_DATA3 12  // B3
#define EXAMPLE_PIN_NUM_DATA4 45  // B4
#define EXAMPLE_PIN_NUM_DATA5 16  // G0
#define EXAMPLE_PIN_NUM_DATA6 17  // G1
#define EXAMPLE_PIN_NUM_DATA7 18  // G2
#define EXAMPLE_PIN_NUM_DATA8 8   // G3
#define EXAMPLE_PIN_NUM_DATA9 3   // G4
#define EXAMPLE_PIN_NUM_DATA10 46 // G5
#define EXAMPLE_PIN_NUM_DATA11 4  // R0
#define EXAMPLE_PIN_NUM_DATA12 5  // R1
#define EXAMPLE_PIN_NUM_DATA13 6  // R2
#define EXAMPLE_PIN_NUM_DATA14 7  // R3
#define EXAMPLE_PIN_NUM_DATA15 15 // R4
#define EXAMPLE_PIN_NUM_DISP_EN -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 320
#define EXAMPLE_LCD_V_RES 240

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB 2
#else
#define EXAMPLE_LCD_NUM_FB 1
#endif // CONFIG_EXAMPLE_DOUBLE_FB

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------3DRender--------------------------------//
static TaskHandle_t xRotationCaculateTask = NULL;
static TaskHandle_t xCalculate3DTask = NULL;
static lv_disp_drv_t disp_drv;

lv_color_t *buffer1 = NULL;
lv_color_t *buffer = NULL;
lv_color_t Black;

Point3 ScreenPoints[] = {{-0.8, 0.8, -1}, {-0.8, -0.8, -1}, {0.8, 0.8, -1}, {0.8, -0.8, -1}}; //屏幕三维对应点： 左上、左下、右上、右下
Point3 ScreenPoints3[] = {{0, 1, -1}, {0, 0, -1}, {1, 1, -1}, {1, 0, -1}}; // 贴图对应点：左上、左下、右上、右下
//--------------------------------3DRender--------------------------------//



// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

extern void example_lvgl_demo_ui(lv_disp_t *disp);

static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE)
    {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    // int offsetx1 = area->x1;
    // int offsetx2 = area->x2;
    // int offsety1 = area->y1;
    // int offsety2 = area->y2;

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    buffer1 = color_map;

    xTaskNotifyGive(xCalculate3DTask);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, 320, 240, buffer);
    lv_disp_flush_ready(drv);
}






//--------------------------------Calculate3DTask--------------------------------//
void Calculate3DTask(void *pvParam)
{
    // lv_disp_drv_t *drv=&disp_drv;
    while (1)
    {
        ESP_LOGI(TAG, "Calculate3DTask:Waiting...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Calculate3DTask:Caculate");

        // 三角形顶点坐标
        Point3 p0 = perspectiveProjection(ScreenPoints[3]);
        Point3 p1 = perspectiveProjection(ScreenPoints[0]);
        Point3 p2 = perspectiveProjection(ScreenPoints[2]);

        // 纹理坐标
        Point3 uv0 = ScreenPoints3[3];
        Point3 uv1 = ScreenPoints3[0];
        Point3 uv2 = ScreenPoints3[2];
        // 三角形顶点坐标
        Point3 ap0 = perspectiveProjection(ScreenPoints[0]);
        Point3 ap1 = perspectiveProjection(ScreenPoints[1]);
        Point3 ap2 = perspectiveProjection(ScreenPoints[3]);

        // 纹理坐标
        Point3 auv0 = ScreenPoints3[0];
        Point3 auv1 = ScreenPoints3[1];
        Point3 auv2 = ScreenPoints3[3];

        // 初始化插值参数``
        float alpha, beta, gamma, alpha1, beta1, gamma1;

        for (int i = 0; i < 240; i++)
        {
            for (int j = 0; j < 320; j++)
            {
                // 计算重心坐标

                alpha = (-(j - p1.x) * (p2.y - p1.y) + (i - p1.y) * (p2.x - p1.x)) / (-(p0.x - p1.x) * (p2.y - p1.y) + (p0.y - p1.y) * (p2.x - p1.x));
                beta = (-(j - p2.x) * (p0.y - p2.y) + (i - p2.y) * (p0.x - p2.x)) / (-(p1.x - p2.x) * (p0.y - p2.y) + (p1.y - p2.y) * (p0.x - p2.x));
                gamma = 1.0f - alpha - beta;
                alpha1 = (-(j - ap1.x) * (ap2.y - ap1.y) + (i - ap1.y) * (ap2.x - ap1.x)) / (-(ap0.x - ap1.x) * (ap2.y - ap1.y) + (ap0.y - ap1.y) * (ap2.x - ap1.x));
                beta1 = (-(j - ap2.x) * (ap0.y - ap2.y) + (i - ap2.y) * (ap0.x - ap2.x)) / (-(ap1.x - ap2.x) * (ap0.y - ap2.y) + (ap1.y - ap2.y) * (ap0.x - ap2.x));
                gamma1 = 1.0f - alpha1 - beta1;

                if (alpha >= 0.0f && beta >= 0.0f && gamma >= 0.0f)
                {
                    float aa = alpha / p0.z;
                    float bb = beta / p1.z;
                    float cc = gamma / p2.z;
                    float zz = 1.0 / (aa + bb + cc);
                    // 纹理坐标在三角形内部
                    float u = zz * (aa * uv0.x + bb * uv1.x + cc * uv2.x);
                    float v = zz * (aa * uv0.y + bb * uv1.y + cc * uv2.y);

                    // 纹理坐标限制在[0, 1]范围内
                    u = fminf(fmaxf(u, 0.0f), 1.0f);
                    v = fminf(fmaxf(v, 0.0f), 1.0f);

                    // 在纹理贴图上查找颜色值
                    int texX = (int)(u * (320 - 1));
                    int texY = (int)(v * (240 - 1));

                    // 设置像素颜色
                    buffer[i * 320 + j] = buffer1[texY * 320 + texX];
                }
                else if (alpha1 >= 0.0f && beta1 >= 0.0f && gamma1 >= 0.0f)
                {
                    // 纹理坐标在三角形内部
                    float aa1 = alpha1 / ap0.z;
                    float bb1 = beta1 / ap1.z;
                    float cc1 = gamma1 / ap2.z;
                    float zz1 = 1.0 / (aa1 + bb1 + cc1);
                    float u = zz1 * (aa1 * auv0.x + bb1 * auv1.x + cc1 * auv2.x);
                    float v = zz1 * (aa1 * auv0.y + bb1 * auv1.y + cc1 * auv2.y);
                    // 纹理坐标限制在[0, 1]范围内
                    u = fminf(fmaxf(u, 0.0f), 1.0f);
                    v = fminf(fmaxf(v, 0.0f), 1.0f);

                    // 在纹理贴图上查找颜色值
                    int texX = (int)(u * (320 - 1));
                    int texY = (int)(v * (240 - 1));

                    // 设置像素颜色
                    buffer[i * 320 + j] = buffer1[texY * 320 + texX];
                }
                else
                {
                    buffer[i * 320 + j] = Black;
                }
            }
        }
    }
    // lv_disp_flush_ready(drv);
}
//--------------------------------Calculate3DTask--------------------------------//



//--------------------------------RotationCaculateTask--------------------------------//
void RotationCaculateTask(void *pvParam)
{
    float cy;
    float sy;
    float cp;
    float sp;
    float cr;
    float sr;
    while (1)
    {
        ESP_LOGI(TAG, "RotationCaculateTask:Waiting...");
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "RotationCaculateTask:Caculate");
        cy = cos(fac_yaw);
        sy = sin(fac_yaw);
        cp = cos(fac_pitch);
        sp = sin(fac_pitch);
        cr = cos(fac_roll);
        sr = sin(fac_roll);
        fac_a = (cp * cr * cr * cy + cp * cy * sr * sr);
        fac_b = (cy * sp * sr - cr * sy);
        fac_c = (cr * cy * sp + sr * sy);
        fac_d = (cp * cr * cr * sy + cp * sr * sr * sy);
        fac_e = (cr * cy + sp * sr * sy);
        fac_f = (-cy * sr + cr * sp * sy);
        fac_g = (cr * cr * cy * cy * sp + cy * cy * sp * sr * sr + cr * cr * sp * sy * sy + sp * sr * sr * sy * sy);
        fac_h = (cp * sr);
        fac_i = (cp * cr);
    }
}
//--------------------------------RotationCaculateTask--------------------------------//





void app_main(void)
{
    // 3D渲染设置
    // UpdateRotation(1, 0.2, 0.2);
    Black = lv_color_make(0, 0, 0);
    buffer = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    initI2C();
    initLcosI2CComand();

    xTaskCreatePinnedToCore(RotationCaculateTask, "RotationCaculateTask", 1024 * 10, NULL, 2, &xRotationCaculateTask, 1);
    fac_yaw = 1;
    fac_pitch = 0.2;
    fac_roll = 0.2;
    xTaskNotifyGive(xRotationCaculateTask);
    xTaskCreatePinnedToCore(Calculate3DTask, "Calculate3DTask", 1024 * 50, NULL, 1, &xCalculate3DTask, 1);

    // 屏幕驱动
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    // static lv_disp_drv_t disp_drv;      // contains callback functions

#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    // LCD面板驱动初始化
    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_back_porch = 80,
            .hsync_front_porch = 40,
            .hsync_pulse_width = 8,
            .vsync_back_porch = 60,
            .vsync_front_porch = 10,
            .vsync_pulse_width = 8,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };

    // 设置屏幕翻转
    //  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle,1,1));
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;

    disp_drv.full_refresh = true;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    // 初始化lvgl定时器
    ESP_LOGI(TAG, "Install LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    // 启动lvgl程序
    ESP_LOGI(TAG, "Display LVGL Scatter Chart");
    example_lvgl_demo_ui(disp);
    float ia = 0;
    while (1)
    {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
        // UpdateRotation(ia/100.0,ia/100.0,ia/100.0);
        fac_yaw = ia / 100.0;
        fac_pitch = ia / 100.0;
        fac_roll = ia / 100.0;
        xTaskNotifyGive(xRotationCaculateTask);
        ia++;
        ESP_LOGI(TAG, "%f", ia);
        if (ia == 40)
        {
            ia = 0;
        }
    }
}
