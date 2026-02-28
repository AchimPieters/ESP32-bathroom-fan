/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   for more information visit https://www.studiopieters.nl
   Professional Bathroom Ventilation Controller
 **/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_system.h>
#include <nvs.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/i2c.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include <button.h>
#include "sht3x.h"

// ==================================================
// GPIO CONFIG
// ==================================================

#define LED_GPIO     CONFIG_ESP_LED_GPIO
#define BUTTON_GPIO  CONFIG_ESP_BUTTON_GPIO

#define FAN_LOW_GPIO   CONFIG_ESP_FAN_LOW_GPIO
#define FAN_MED_GPIO   CONFIG_ESP_FAN_MED_GPIO
#define FAN_HIGH_GPIO  CONFIG_ESP_FAN_HIGH_GPIO

// ==================================================
// I2C
// ==================================================

#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_SCL  CONFIG_I2C_MASTER_SCL
#define I2C_MASTER_SDA  CONFIG_I2C_MASTER_SDA
#define SHT3X_ADDR      CONFIG_SHT3X_I2C_ADDRESS

static esp_err_t i2c_master_init(void)
{
        i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = I2C_MASTER_SDA,
                .scl_io_num = I2C_MASTER_SCL,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = 100000
        };

        ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
        return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// ==================================================
// ISO PARAMETERS
// ==================================================

#define HUM_ON      65.0f
#define HUM_OFF     60.0f
#define HUM_RISE     8.0f

#define TEMP_ON     26.0f
#define TEMP_OFF    24.0f

#define MIN_RUNTIME_MIN   15
#define MANUAL_TIMEOUT_MIN 20

#define EMA_ALPHA 0.2f

// ==================================================
// HAP SAFE LIMITS
// ==================================================

#define TEMP_NOTIFY_DELTA  0.2f
#define HUM_NOTIFY_DELTA   0.5f

#define TEMP_MIN_NOTIFY_MS        30000
#define HUM_MIN_NOTIFY_MS_NORMAL  15000
#define HUM_MIN_NOTIFY_MS_SPIKE    5000

#define MAX_EVENTS_PER_MIN_NORMAL 20
#define MAX_EVENTS_PER_MIN_SPIKE  35

#define HUM_SPIKE_LEVEL 70.0f

// ==================================================
// STATE
// ==================================================

typedef enum {
        FAN_OFF = 0,
        FAN_LOW,
        FAN_MID,
        FAN_HIGH
} fan_mode_t;

static fan_mode_t fan_mode = FAN_OFF;

static float ema_temp = 0;
static float ema_hum  = 0;
static float baseline_humidity = 0;

static float last_notified_temp = 0;
static float last_notified_hum  = 0;

static TickType_t last_temp_notify = 0;
static TickType_t last_hum_notify  = 0;

static TickType_t minute_window_start = 0;
static uint32_t events_this_minute = 0;

static bool auto_mode = true;
static TickType_t fan_started_at = 0;
static TickType_t manual_started_at = 0;

// ==================================================
// FAN CONTROL
// ==================================================

static void pulse(gpio_num_t pin)
{
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(pin, 1);
}

static void fan_set(fan_mode_t mode)
{
        if (fan_mode == mode) return;
        fan_mode = mode;

        switch (mode) {
        case FAN_OFF:  pulse(FAN_LOW_GPIO); break;
        case FAN_LOW:  pulse(FAN_LOW_GPIO); break;   // 30%
        case FAN_MID:  pulse(FAN_MED_GPIO); break;   // 60%
        case FAN_HIGH: pulse(FAN_HIGH_GPIO); break;  // 100%
        }
}

// ==================================================
// IDENTIFY LED
// ==================================================

void accessory_identify(homekit_value_t _value)
{
        for (int i = 0; i < 3; i++) {
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(150));
                gpio_set_level(LED_GPIO, 0);
                vTaskDelay(pdMS_TO_TICKS(150));
        }
}

// ==================================================
// BUTTON CALLBACK
// ==================================================

void button_callback(button_event_t event, void *context)
{
        switch (event) {

        case button_event_single_press:
                lifecycle_request_update_and_reboot();
                break;

        case button_event_double_press:
                homekit_server_reset();
                esp_restart();
                break;

        case button_event_long_press:
                lifecycle_factory_reset_and_reboot();
                break;

        default:
                break;
        }
}

// ==================================================
// SAFE NOTIFY
// ==================================================

static void safe_notify(
        homekit_characteristic_t *ch,
        homekit_value_t value,
        TickType_t *last_tick,
        uint32_t min_interval_ms,
        uint32_t max_events_per_min)
{
        TickType_t now = xTaskGetTickCount();

        if ((now - minute_window_start) > pdMS_TO_TICKS(60000)) {
                minute_window_start = now;
                events_this_minute = 0;
        }

        if (events_this_minute >= max_events_per_min) return;
        if ((now - *last_tick) < pdMS_TO_TICKS(min_interval_ms)) return;

        homekit_characteristic_notify(ch, value);

        *last_tick = now;
        events_this_minute++;
}

// ==================================================
// HOMEKIT FAN
// ==================================================

static bool hk_fan_on = false;
static float hk_speed = 0;

homekit_value_t fan_on_get() {
        return HOMEKIT_BOOL(hk_fan_on);
}

void fan_on_set(homekit_value_t value)
{
        hk_fan_on = value.bool_value;
        auto_mode = false;
        manual_started_at = xTaskGetTickCount();

        if (!hk_fan_on) {
                fan_set(FAN_OFF);
        }
}

homekit_value_t fan_speed_get() {
        return HOMEKIT_FLOAT(hk_speed);
}

void fan_speed_set(homekit_value_t value)
{
        hk_speed = value.float_value;
        auto_mode = false;
        manual_started_at = xTaskGetTickCount();

        if (hk_speed < 40) {
                fan_set(FAN_LOW);
        } else if (hk_speed < 80) {
                fan_set(FAN_MID);
        } else {
                fan_set(FAN_HIGH);
        }

        hk_fan_on = true;
}

// ==================================================
// HOMEKIT CHARACTERISTICS
// ==================================================

homekit_characteristic_t fan_on_characteristic =
        HOMEKIT_CHARACTERISTIC_(ON, false,
                                .getter = fan_on_get,
                                .setter = fan_on_set);

homekit_characteristic_t fan_speed_characteristic =
        HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0,
                                .getter = fan_speed_get,
                                .setter = fan_speed_set);

homekit_characteristic_t temperature_characteristic =
        HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);

homekit_characteristic_t humidity_characteristic =
        HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

homekit_characteristic_t revision =
        HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);

homekit_characteristic_t ota_trigger = API_OTA_TRIGGER;

// ==================================================
// SENSOR TASK
// ==================================================

static void sensor_task(void *arg)
{
        while (1)
        {
                float t=0, h=0;

                if (sht3x_read_temperature_humidity(SHT3X_ADDR,&t,&h)==ESP_OK)
                {
                        ema_temp = (ema_temp==0) ? t : EMA_ALPHA*t + (1-EMA_ALPHA)*ema_temp;
                        ema_hum  = (ema_hum==0) ?  h : EMA_ALPHA*h + (1-EMA_ALPHA)*ema_hum;

                        if (fan_mode == FAN_OFF)
                                baseline_humidity =
                                        (baseline_humidity==0) ?
                                        ema_hum : baseline_humidity*0.98f + ema_hum*0.02f;

                        float rise = ema_hum - baseline_humidity;

                        bool spike_mode =
                                (ema_hum >= HUM_SPIKE_LEVEL) ||
                                (rise > HUM_RISE);

                        uint32_t max_epm = spike_mode ?
                                           MAX_EVENTS_PER_MIN_SPIKE :
                                           MAX_EVENTS_PER_MIN_NORMAL;

                        uint32_t hum_interval = spike_mode ?
                                                HUM_MIN_NOTIFY_MS_SPIKE :
                                                HUM_MIN_NOTIFY_MS_NORMAL;

                        // AUTO CONTROL

                        if (auto_mode)
                        {
                                if (ema_hum > HUM_ON ||
                                    rise > HUM_RISE ||
                                    ema_temp > TEMP_ON)
                                {
                                        fan_set(FAN_HIGH);
                                        fan_started_at = xTaskGetTickCount();
                                }

                                uint32_t runtime =
                                        (xTaskGetTickCount()-fan_started_at)/
                                        pdMS_TO_TICKS(60000);

                                if (runtime > MIN_RUNTIME_MIN &&
                                    ema_hum < HUM_OFF &&
                                    ema_temp < TEMP_OFF)
                                {
                                        fan_set(FAN_LOW);
                                }
                        }
                        else
                        {
                                uint32_t manual_runtime =
                                        (xTaskGetTickCount()-manual_started_at)/
                                        pdMS_TO_TICKS(60000);

                                if (manual_runtime > MANUAL_TIMEOUT_MIN)
                                {
                                        auto_mode = true;
                                }
                        }

                        // SAFE NOTIFY

                        if (fabsf(ema_temp-last_notified_temp)>TEMP_NOTIFY_DELTA)
                        {
                                safe_notify(&temperature_characteristic,
                                            HOMEKIT_FLOAT(ema_temp),
                                            &last_temp_notify,
                                            TEMP_MIN_NOTIFY_MS,
                                            max_epm);

                                last_notified_temp = ema_temp;
                        }

                        if (fabsf(ema_hum-last_notified_hum)>HUM_NOTIFY_DELTA)
                        {
                                safe_notify(&humidity_characteristic,
                                            HOMEKIT_FLOAT(ema_hum),
                                            &last_hum_notify,
                                            hum_interval,
                                            max_epm);

                                last_notified_hum = ema_hum;
                        }

                        vTaskDelay(pdMS_TO_TICKS(spike_mode ? 3000 : 10000));
                }
        }
}

// ==================================================
// MAIN
// ==================================================

void app_main(void)
{
        ESP_ERROR_CHECK(lifecycle_nvs_init());

        gpio_reset_pin(LED_GPIO);
        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(LED_GPIO, 0);

        gpio_reset_pin(FAN_LOW_GPIO);
        gpio_set_direction(FAN_LOW_GPIO, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(FAN_LOW_GPIO, 1);

        gpio_reset_pin(FAN_MED_GPIO);
        gpio_set_direction(FAN_MED_GPIO, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(FAN_MED_GPIO, 1);

        gpio_reset_pin(FAN_HIGH_GPIO);
        gpio_set_direction(FAN_HIGH_GPIO, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(FAN_HIGH_GPIO, 1);

        button_config_t btn_cfg = button_config_default(button_active_low);
        button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL);

        ESP_ERROR_CHECK(i2c_master_init());
        ESP_ERROR_CHECK(sht3x_init(SHT3X_ADDR));

        xTaskCreate(sensor_task,"sensor",4096,NULL,5,NULL);

        wifi_start(NULL);
}
