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

 #include <math.h>
 #include <stdbool.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <time.h>
 #include <sys/time.h>

 #include <driver/gpio.h>
 #include <driver/i2c.h>
 #include <esp_err.h>
 #include <esp_log.h>
 #include <esp_system.h>
 #include <esp_netif_sntp.h>
 #include <freertos/FreeRTOS.h>
 #include <freertos/task.h>
 #include <homekit/characteristics.h>
 #include <homekit/homekit.h>
 #include <nvs.h>

 #include <button.h>
 #include <sht3x.h>

 #include "esp32-lcm.h"

// ==================================================
// GPIO CONFIGURATION
// ==================================================

 #define LED_GPIO CONFIG_ESP_LED_GPIO
 #define BUTTON_GPIO CONFIG_ESP_BUTTON_GPIO

 #define FAN_LOW_GPIO CONFIG_ESP_FAN_LOW_GPIO
 #define FAN_MED_GPIO CONFIG_ESP_FAN_MED_GPIO
 #define FAN_HIGH_GPIO CONFIG_ESP_FAN_HIGH_GPIO

// ==================================================
// I2C CONFIGURATION
// ==================================================

 #define I2C_MASTER_PORT I2C_NUM_0
 #define I2C_MASTER_SCL CONFIG_I2C_MASTER_SCL
 #define I2C_MASTER_SDA CONFIG_I2C_MASTER_SDA
 #define SHT3X_ADDR CONFIG_SHT3X_I2C_ADDRESS

static const char *TAG_MAIN = "MAIN";
static const char *TAG_SENSOR = "SENSOR";

static esp_err_t i2c_master_init(void) {
        i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = I2C_MASTER_SDA,
                .scl_io_num = I2C_MASTER_SCL,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = 100000,
        };

        ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &conf));
        return i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
}

// ==================================================
// QUIET-FIRST HUMIDITY CONTROL PARAMETERS
// ==================================================

 #define HUM_OFF 58.0f
 #define HUM_LOW_ON 60.0f
 #define HUM_MID_ON 70.0f
 #define HUM_HIGH_ON 80.0f
 #define HUM_EMERGENCY 85.0f

 #define HUM_RISE_MID 10.0f
 #define HUM_RISE_HIGH 18.0f

 #define SHOWER_MIN_HUM 55.0f
 #define SHOWER_TEMP_RISE_HUM 5.0f

 #define TEMP_ON 30.0f
 #define TEMP_MID_ON 32.0f
 #define TEMP_OFF 28.0f

 #define MIN_RUNTIME_MIN 10
 #define MANUAL_TIMEOUT_MIN 20

 #define EMA_ALPHA 0.2f

// ==================================================
// SENSOR CALIBRATION
// ==================================================

 #define TEMP_OFFSET 0.8f
 #define HUM_OFFSET -10.0f

// ==================================================
// NIGHT QUIET MODE
// ==================================================

 #define NIGHT_START_HOUR 22
 #define NIGHT_END_HOUR 7

 #define NIGHT_MAX_MODE FAN_LOW
 #define NIGHT_EMERGENCY_HUM 85.0f

// ==================================================
// HOMEKIT SAFE NOTIFY LIMITS
// ==================================================

 #define TEMP_NOTIFY_DELTA 0.2f
 #define HUM_NOTIFY_DELTA 0.5f

 #define TEMP_MIN_NOTIFY_MS 30000
 #define HUM_MIN_NOTIFY_MS_NORMAL 15000
 #define HUM_MIN_NOTIFY_MS_SPIKE 5000

 #define MAX_EVENTS_PER_MIN_NORMAL 20
 #define MAX_EVENTS_PER_MIN_SPIKE 35

 #define HUM_SPIKE_LEVEL 70.0f

// ==================================================
// STATE
// ==================================================

typedef enum {
        FAN_OFF = 0,
        FAN_LOW,
        FAN_MID,
        FAN_HIGH,
} fan_mode_t;

static fan_mode_t fan_mode = FAN_OFF;

static float ema_temp = 0;
static float ema_hum = 0;
static float baseline_humidity = 0;

static float last_notified_temp = 0;
static float last_notified_hum = 0;

static TickType_t last_temp_notify = 0;
static TickType_t last_hum_notify = 0;

static TickType_t minute_window_start = 0;
static uint32_t events_this_minute = 0;

static bool auto_mode = true;
static TickType_t fan_started_at = 0;
static TickType_t manual_started_at = 0;

static bool hk_fan_on = false;
static float hk_speed = 0;

static bool sntp_started = false;

// ==================================================
// HOMEKIT CHARACTERISTICS
// ==================================================

static homekit_value_t fan_on_get(void);
static void fan_on_set(homekit_value_t value);
static homekit_value_t fan_speed_get(void);
static void fan_speed_set(homekit_value_t value);

homekit_characteristic_t fan_on_characteristic =
        HOMEKIT_CHARACTERISTIC_(ON, false, .getter = fan_on_get, .setter = fan_on_set);

homekit_characteristic_t fan_speed_characteristic =
        HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0, .getter = fan_speed_get, .setter = fan_speed_set);

homekit_characteristic_t temperature_characteristic =
        HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);

homekit_characteristic_t humidity_characteristic =
        HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "Bathroom Fan");
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER, "StudioPieters®");
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "R1TFL8J965HE");
homekit_characteristic_t model = HOMEKIT_CHARACTERISTIC_(MODEL, "TY3V0LC/Q");
homekit_characteristic_t revision =
        HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);
homekit_characteristic_t ota_trigger = API_OTA_TRIGGER;

// ==================================================
// TIME / SNTP
// ==================================================

static void time_sync_start(void) {
        if (sntp_started) {
                return;
        }

        setenv("TZ", "CET-1CEST,M3.5.0/2,M10.5.0/3", 1);
        tzset();

        esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_err_t err = esp_netif_sntp_init(&config);

        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
                sntp_started = true;
                ESP_LOGI(TAG_MAIN, "SNTP started");
        } else {
                ESP_LOGW(TAG_MAIN, "SNTP start failed: %s", esp_err_to_name(err));
        }
}

static bool is_night_time(void) {
        time_t now = 0;
        struct tm timeinfo = {0};

        time(&now);

        if (now < 1700000000) {
                return false;
        }

        localtime_r(&now, &timeinfo);

        int hour = timeinfo.tm_hour;

        if (NIGHT_START_HOUR > NIGHT_END_HOUR) {
                return hour >= NIGHT_START_HOUR || hour < NIGHT_END_HOUR;
        }

        return hour >= NIGHT_START_HOUR && hour < NIGHT_END_HOUR;
}

// ==================================================
// FAN CONTROL
// ==================================================

static void pulse(gpio_num_t pin) {
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(pin, 1);
}

static void fan_set(fan_mode_t mode) {
        if (fan_mode == mode) {
                return;
        }

        fan_mode = mode;

        switch (mode) {
        case FAN_OFF:
                pulse(FAN_LOW_GPIO);
                hk_fan_on = false;
                hk_speed = 0;
                break;

        case FAN_LOW:
                pulse(FAN_LOW_GPIO);
                hk_fan_on = true;
                hk_speed = 30;
                break;

        case FAN_MID:
                pulse(FAN_MED_GPIO);
                hk_fan_on = true;
                hk_speed = 60;
                break;

        case FAN_HIGH:
                pulse(FAN_HIGH_GPIO);
                hk_fan_on = true;
                hk_speed = 100;
                break;
        }

        fan_on_characteristic.value = HOMEKIT_BOOL(hk_fan_on);
        fan_speed_characteristic.value = HOMEKIT_FLOAT(hk_speed);

        homekit_characteristic_notify(&fan_on_characteristic, fan_on_characteristic.value);
        homekit_characteristic_notify(&fan_speed_characteristic, fan_speed_characteristic.value);
}

// ==================================================
// IDENTIFY LED
// ==================================================

void accessory_identify(homekit_value_t value) {
        (void)value;

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

void button_callback(button_event_t event, void *context) {
        (void)context;

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
// SAFE HOMEKIT NOTIFY
// ==================================================

static void safe_notify(homekit_characteristic_t *ch,
                        homekit_value_t value,
                        TickType_t *last_tick,
                        uint32_t min_interval_ms,
                        uint32_t max_events_per_min) {
        TickType_t now = xTaskGetTickCount();

        if ((now - minute_window_start) > pdMS_TO_TICKS(60000)) {
                minute_window_start = now;
                events_this_minute = 0;
        }

        if (events_this_minute >= max_events_per_min) {
                return;
        }

        if ((now - *last_tick) < pdMS_TO_TICKS(min_interval_ms)) {
                return;
        }

        homekit_characteristic_notify(ch, value);

        *last_tick = now;
        events_this_minute++;
}

// ==================================================
// HOMEKIT FAN
// ==================================================

static homekit_value_t fan_on_get(void) {
        return HOMEKIT_BOOL(hk_fan_on);
}

static void fan_on_set(homekit_value_t value) {
        if (value.format != homekit_format_bool) {
                return;
        }

        hk_fan_on = value.bool_value;
        auto_mode = false;
        manual_started_at = xTaskGetTickCount();

        if (!hk_fan_on) {
                fan_set(FAN_OFF);
                return;
        }

        if (hk_speed <= 0) {
                hk_speed = 30;
        }

        if (hk_speed < 40) {
                fan_set(FAN_LOW);
        } else if (hk_speed < 80) {
                fan_set(FAN_MID);
        } else {
                fan_set(FAN_HIGH);
        }
}

static homekit_value_t fan_speed_get(void) {
        return HOMEKIT_FLOAT(hk_speed);
}

static void fan_speed_set(homekit_value_t value) {
        if (value.format != homekit_format_float && value.format != homekit_format_int) {
                return;
        }

        hk_speed = (value.format == homekit_format_float) ? value.float_value : (float)value.int_value;
        auto_mode = false;
        manual_started_at = xTaskGetTickCount();

        if (hk_speed <= 0) {
                fan_set(FAN_OFF);
        } else if (hk_speed < 40) {
                fan_set(FAN_LOW);
        } else if (hk_speed < 80) {
                fan_set(FAN_MID);
        } else {
                fan_set(FAN_HIGH);
        }
}

// ==================================================
// SENSOR TASK
// ==================================================

static void sensor_task(void *arg) {
        (void)arg;

        while (1) {
                float t = 0;
                float h = 0;

                if (sht3x_read_temperature_humidity(SHT3X_ADDR, &t, &h) == ESP_OK) {
                        t += TEMP_OFFSET;
                        h += HUM_OFFSET;

                        if (h < 0.0f) {
                                h = 0.0f;
                        }

                        if (h > 100.0f) {
                                h = 100.0f;
                        }

                        ema_temp = (ema_temp == 0) ? t : EMA_ALPHA * t + (1 - EMA_ALPHA) * ema_temp;
                        ema_hum = (ema_hum == 0) ? h : EMA_ALPHA * h + (1 - EMA_ALPHA) * ema_hum;

                        if (fan_mode == FAN_OFF) {
                                baseline_humidity =
                                        (baseline_humidity == 0) ? ema_hum : baseline_humidity * 0.98f + ema_hum * 0.02f;
                        }

                        float rise = ema_hum - baseline_humidity;

                        bool spike_mode = (ema_hum >= HUM_SPIKE_LEVEL) || (rise >= HUM_RISE_MID);

                        uint32_t max_epm = spike_mode ? MAX_EVENTS_PER_MIN_SPIKE : MAX_EVENTS_PER_MIN_NORMAL;
                        uint32_t hum_interval = spike_mode ? HUM_MIN_NOTIFY_MS_SPIKE : HUM_MIN_NOTIFY_MS_NORMAL;

                        if (auto_mode) {
                                bool night_mode = is_night_time();

                                fan_mode_t target_mode = FAN_OFF;

                                bool temp_low = ema_temp >= TEMP_ON;
                                bool temp_mid = ema_temp >= TEMP_MID_ON;

                                bool shower_detected =
                                        (rise >= HUM_RISE_MID && ema_hum >= SHOWER_MIN_HUM) ||
                                        (ema_temp >= TEMP_ON && rise >= SHOWER_TEMP_RISE_HUM) ||
                                        (ema_hum >= HUM_MID_ON);

                                bool emergency = ema_hum >= HUM_EMERGENCY || ema_hum >= NIGHT_EMERGENCY_HUM;
                                bool high_needed = emergency || ema_hum >= HUM_HIGH_ON || rise >= HUM_RISE_HIGH;
                                bool mid_needed = shower_detected || temp_mid;
                                bool low_needed = ema_hum >= HUM_LOW_ON || temp_low;

                                if (high_needed) {
                                        target_mode = FAN_HIGH;
                                } else if (mid_needed) {
                                        target_mode = FAN_MID;
                                } else if (low_needed) {
                                        target_mode = FAN_LOW;
                                } else {
                                        target_mode = FAN_OFF;
                                }

                                if (night_mode && !emergency && target_mode > NIGHT_MAX_MODE) {
                                        target_mode = NIGHT_MAX_MODE;
                                }

                                if (fan_mode == FAN_OFF && target_mode != FAN_OFF) {
                                        fan_started_at = xTaskGetTickCount();
                                }

                                uint32_t runtime =
                                        (xTaskGetTickCount() - fan_started_at) / pdMS_TO_TICKS(60000);

                                if (fan_mode != FAN_OFF && target_mode == FAN_OFF && runtime < MIN_RUNTIME_MIN) {
                                        target_mode = FAN_LOW;
                                }

                                if (fan_mode != FAN_OFF &&
                                    runtime >= MIN_RUNTIME_MIN &&
                                    ema_hum < HUM_OFF &&
                                    ema_temp < TEMP_OFF) {
                                        target_mode = FAN_OFF;
                                }

                                if (target_mode != fan_mode) {
                                        fan_set(target_mode);
                                }

                                ESP_LOGI(TAG_SENSOR,
                                         "Control: Shower=%d Emergency=%d Low=%d Mid=%d High=%d Target=%d",
                                         shower_detected,
                                         emergency,
                                         low_needed,
                                         mid_needed,
                                         high_needed,
                                         target_mode);
                        } else {
                                uint32_t manual_runtime =
                                        (xTaskGetTickCount() - manual_started_at) / pdMS_TO_TICKS(60000);

                                if (manual_runtime > MANUAL_TIMEOUT_MIN) {
                                        auto_mode = true;
                                }
                        }

                        if (fabsf(ema_temp - last_notified_temp) > TEMP_NOTIFY_DELTA) {
                                temperature_characteristic.value = HOMEKIT_FLOAT(ema_temp);
                                safe_notify(&temperature_characteristic,
                                            temperature_characteristic.value,
                                            &last_temp_notify,
                                            TEMP_MIN_NOTIFY_MS,
                                            max_epm);

                                last_notified_temp = ema_temp;
                        }

                        if (fabsf(ema_hum - last_notified_hum) > HUM_NOTIFY_DELTA) {
                                humidity_characteristic.value = HOMEKIT_FLOAT(ema_hum);
                                safe_notify(&humidity_characteristic,
                                            humidity_characteristic.value,
                                            &last_hum_notify,
                                            hum_interval,
                                            max_epm);

                                last_notified_hum = ema_hum;
                        }

                        ESP_LOGI(TAG_SENSOR,
                                 "Temp=%.2fC Hum=%.2f%% Baseline=%.2f%% Rise=%.2f%% Auto=%d Mode=%d Night=%d",
                                 ema_temp,
                                 ema_hum,
                                 baseline_humidity,
                                 rise,
                                 auto_mode,
                                 fan_mode,
                                 is_night_time());

                        vTaskDelay(pdMS_TO_TICKS(spike_mode ? 3000 : 10000));
                } else {
                        ESP_LOGW(TAG_SENSOR, "SHT3X read failed, retrying in 2s");
                        vTaskDelay(pdMS_TO_TICKS(2000));
                }
        }
}

// ==================================================
// HOMEKIT ACCESSORY CONFIGURATION
// ==================================================

 #pragma GCC diagnostic push
 #pragma GCC diagnostic ignored "-Woverride-init"

homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(.id = 1,
                          .category = homekit_accessory_category_fan,
                          .services =
                                  (homekit_service_t *[]) {
                HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                                .characteristics =
                                        (homekit_characteristic_t *[]) {
                        &name,
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY,
                                               accessory_identify),
                        NULL,
                }),

                HOMEKIT_SERVICE(FAN,
                                .primary = true,
                                .characteristics =
                                        (homekit_characteristic_t *[]) {
                        HOMEKIT_CHARACTERISTIC(NAME,
                                               "Bathroom Fan"),
                        &fan_on_characteristic,
                        &fan_speed_characteristic,
                        &ota_trigger,
                        NULL,
                }),

                HOMEKIT_SERVICE(TEMPERATURE_SENSOR,
                                .characteristics =
                                        (homekit_characteristic_t *[]) {
                        HOMEKIT_CHARACTERISTIC(NAME,
                                               "Bathroom Temperature"),
                        &temperature_characteristic,
                        NULL,
                }),

                HOMEKIT_SERVICE(HUMIDITY_SENSOR,
                                .characteristics =
                                        (homekit_characteristic_t *[]) {
                        HOMEKIT_CHARACTERISTIC(
                                NAME,
                                "Bathroom Humidity"),
                        &humidity_characteristic,
                        NULL,
                }),

                NULL,
        }),
        NULL,
};

 #pragma GCC diagnostic pop

homekit_server_config_t config = {
        .accessories = accessories,
        .password = CONFIG_ESP_SETUP_CODE,
        .setupId = CONFIG_ESP_SETUP_ID,
};

static void on_wifi_ready(void) {
        static bool homekit_started = false;

        time_sync_start();

        if (homekit_started) {
                return;
        }

        homekit_server_init(&config);
        homekit_started = true;
}

// ==================================================
// MAIN
// ==================================================

void app_main(void) {
        ESP_ERROR_CHECK(lifecycle_nvs_init());
        lifecycle_log_post_reset_state(TAG_MAIN);
        ESP_ERROR_CHECK(lifecycle_configure_homekit(&revision, &ota_trigger, TAG_MAIN));

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

        gpio_reset_pin(BUTTON_GPIO);
        gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
        gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);

        button_config_t btn_cfg = {
                .active_level         = button_active_low,
                .long_press_time      = 2000,
                .repeat_press_timeout = 400,
                .max_repeat_presses   = 3,
        };

        if (button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL)) {
                ESP_LOGE(TAG_MAIN, "Button init failed");
        } else {
                ESP_LOGI(TAG_MAIN,
                         "Button initialized on GPIO%d as active-low with internal pull-up",
                         BUTTON_GPIO);
        }

        ESP_ERROR_CHECK(i2c_master_init());
        ESP_ERROR_CHECK(sht3x_init(SHT3X_ADDR));

        xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);

        esp_err_t wifi_err = wifi_start(on_wifi_ready);
        if (wifi_err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGW(TAG_MAIN, "WiFi provisioning data missing");
        } else if (wifi_err != ESP_OK) {
                ESP_LOGE(TAG_MAIN, "wifi_start failed: %s", esp_err_to_name(wifi_err));
        }
}
