// main.c
/**
   Copyright 2026 Achim Pieters | StudioPieters®
   (MIT License text unchanged)
 **/

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <driver/i2c.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include <button.h>

#include "sht3x.h"

#define BUTTON_GPIO CONFIG_ESP_BUTTON_GPIO
#define LED_GPIO    CONFIG_ESP_LED_GPIO
static const char *TAG = "FAN_REMOTE";

// Remote "button" outputs (each gives one pulse)
#define FAN_LOW_GPIO  CONFIG_ESP_FAN_LOW_GPIO
#define FAN_MED_GPIO  CONFIG_ESP_FAN_MED_GPIO
#define FAN_HIGH_GPIO CONFIG_ESP_FAN_HIGH_GPIO

// --------- HARD CODED TIMING (NOT IN menuconfig) ---------
#define PULSE_MS   200
#define LOCKOUT_MS 600
// ---------------------------------------------------------

// ----------------- HUMIDITY AUTO CONTROL (SHT3x) -----------------
#define HUM_SAMPLE_MS           2000

#define BASELINE_ALPHA          0.02f

#define RH_ABS_MED_ON           60.0f
#define RH_ABS_HIGH_ON          72.0f

#define RH_BASE_PLUS_MED        8.0f
#define RH_BASE_PLUS_HIGH       15.0f

#define RH_MED_OFF_HYST         3.0f
#define RH_HIGH_OFF_HYST        4.0f

#define RH_RISE_HIGH_PER_MIN    2.0f
#define RH_RISE_MED_PER_MIN     1.0f

#define HIGH_MIN_RUN_MS         (10 * 60 * 1000)
#define MED_MIN_RUN_MS          (5  * 60 * 1000)

#define MANUAL_HOLD_MS          (20 * 60 * 1000)
// -----------------------------------------------------------------

// ----- Slider UX constants -----
#define SPEED_LOW_MAX   33.0f
#define SPEED_MED_MAX   66.0f

// Active ON while LOW: jump to 34% (MED range)
#define SPEED_ON_JUMP_TO_MED 34.0f

// ----- HomeKit sensor notify thresholds (reduce spam) -----
#define TEMP_NOTIFY_DELTA_C   0.1f
#define RH_NOTIFY_DELTA_PCT   0.5f

// ----- State -----
typedef enum {
        FAN_MODE_LOW  = 0, // level 1 = low = "off"
        FAN_MODE_MED  = 1, // level 2 = medium
        FAN_MODE_HIGH = 2  // level 3 = high
} fan_mode_t;

static fan_mode_t fan_mode = FAN_MODE_LOW;

// Fan tile should be OFF after reboot (but physically fan is LOW)
static uint8_t active_u8 = 0;

// Default: AUTOMATIC after boot
static bool auto_enabled = true;

static int64_t last_pulse_ms = 0;
static bool boot_low_pulsed = false;

// ---- Humidity control state ----
static float rh_now = NAN;
static float t_now  = NAN;

static float rh_base = 50.0f;
static bool rh_base_valid = false;

static int64_t last_sample_ms = 0;
static float last_rh = NAN;

static int64_t mode_enter_ms = 0;
static int64_t manual_hold_until_ms = 0;

static uint8_t g_sht3x_addr = 0x44;

// HomeKit characteristics
#define DEVICE_NAME "Bathroom Fan"
#define DEVICE_MANUFACTURER "StudioPieters®"
#define DEVICE_SERIAL "TY3V0LC/Q"
#define DEVICE_MODEL "R1TFL8J965HE"

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  DEVICE_MANUFACTURER);
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
homekit_characteristic_t model = HOMEKIT_CHARACTERISTIC_(MODEL, DEVICE_MODEL);
homekit_characteristic_t revision = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);

// Keep OTA trigger available via lifecycle_configure_homekit,
// but do NOT attach it to primary services for iOS strictness.
homekit_characteristic_t ota_trigger = API_OTA_TRIGGER;

// ---- FAN characteristics ----
homekit_characteristic_t ch_fan_active        = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);            // uint8 0/1
homekit_characteristic_t ch_current_fan_state = HOMEKIT_CHARACTERISTIC_(CURRENT_FAN_STATE, 0); // 0/2
homekit_characteristic_t ch_target_fan_state  = HOMEKIT_CHARACTERISTIC_(TARGET_FAN_STATE, 1);  // 0=manual,1=auto
homekit_characteristic_t ch_rotation_speed    = HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0.0f); // float 0..100

// ---- SHT3x sensor characteristics (visible in HomeKit) ----
homekit_characteristic_t ch_current_temperature =
        HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0.0f);
homekit_characteristic_t ch_current_humidity =
        HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0.0f);

// ----- Helpers -----
static void led_write(bool on) {
        gpio_set_level(LED_GPIO, on ? 1 : 0);
}

static int64_t now_ms(void) {
        return esp_timer_get_time() / 1000;
}

static void press_pulse(gpio_num_t pin, int pulse_ms) {
        int64_t t = now_ms();

        if (LOCKOUT_MS > 0 && (t - last_pulse_ms < LOCKOUT_MS)) {
                ESP_LOGW(TAG, "Pulse ignored (lockout)");
                return;
        }
        last_pulse_ms = t;

        ESP_LOGI(TAG, "Pulse GPIO %d for %dms (active-low, open-drain)", (int)pin, pulse_ms);

        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(pulse_ms));
        gpio_set_level(pin, 1);
}

static void init_remote_button_gpio(gpio_num_t pin) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD);
        gpio_set_level(pin, 1); // released (Hi-Z)
}

static void remote_pulse_mode(fan_mode_t mode) {
        switch (mode) {
        case FAN_MODE_LOW:
                press_pulse(FAN_LOW_GPIO, PULSE_MS);
                ESP_LOGI(TAG, "Remote pulse: LOW (level 1 / OFF-default)");
                break;
        case FAN_MODE_MED:
                press_pulse(FAN_MED_GPIO, PULSE_MS);
                ESP_LOGI(TAG, "Remote pulse: MED (level 2)");
                break;
        case FAN_MODE_HIGH:
                press_pulse(FAN_HIGH_GPIO, PULSE_MS);
                ESP_LOGI(TAG, "Remote pulse: HIGH (level 3)");
                break;
        default:
                press_pulse(FAN_LOW_GPIO, PULSE_MS);
                ESP_LOGW(TAG, "Unknown mode -> pulse LOW");
                mode = FAN_MODE_LOW;
                break;
        }

        fan_mode = mode;
        led_write(fan_mode == FAN_MODE_HIGH);
}

static void gpio_init_all(void) {
        gpio_reset_pin(LED_GPIO);
        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
        led_write(false);

        init_remote_button_gpio(FAN_LOW_GPIO);
        init_remote_button_gpio(FAN_MED_GPIO);
        init_remote_button_gpio(FAN_HIGH_GPIO);

        // Default: level 1 (LOW) but HomeKit shows OFF
        fan_mode = FAN_MODE_LOW;
        active_u8 = 0;

        // Default: AUTO after boot
        auto_enabled = true;

        ch_fan_active.value = HOMEKIT_UINT8(active_u8);
        ch_current_fan_state.value = HOMEKIT_UINT8(0);
        ch_target_fan_state.value = HOMEKIT_UINT8(1); // AUTO
        ch_rotation_speed.value = HOMEKIT_FLOAT(0.0f);

        // Sensor defaults
        ch_current_temperature.value = HOMEKIT_FLOAT(0.0f);
        ch_current_humidity.value = HOMEKIT_FLOAT(0.0f);
}

// ---- I2C init for SHT3x ----
#ifndef SHT3X_I2C_PORT
#define SHT3X_I2C_PORT I2C_NUM_0
#endif

#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

static void i2c_master_init(void)
{
        i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = CONFIG_I2C_MASTER_SDA,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_io_num = CONFIG_I2C_MASTER_SCL,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = I2C_MASTER_FREQ_HZ,
        };

        ESP_ERROR_CHECK(i2c_param_config(SHT3X_I2C_PORT, &conf));
        ESP_ERROR_CHECK(i2c_driver_install(SHT3X_I2C_PORT, conf.mode,
                                           I2C_MASTER_RX_BUF_DISABLE,
                                           I2C_MASTER_TX_BUF_DISABLE, 0));

        ESP_LOGI(TAG, "I2C init for SHT3x: port=%d SDA=%d SCL=%d addr=0x%02X",
                 (int)SHT3X_I2C_PORT, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL, (int)g_sht3x_addr);
}

// ----- HomeKit sync helpers -----
static void sync_and_notify_fan(void) {
        ch_fan_active.value = HOMEKIT_UINT8(active_u8);

        if (active_u8 == 0) {
                ch_current_fan_state.value = HOMEKIT_UINT8(0); // Home shows OFF
        } else {
                ch_current_fan_state.value = HOMEKIT_UINT8(2); // blowing
        }

        ch_target_fan_state.value = HOMEKIT_UINT8(auto_enabled ? 1 : 0);

        homekit_characteristic_notify(&ch_fan_active, ch_fan_active.value);
        homekit_characteristic_notify(&ch_current_fan_state, ch_current_fan_state.value);
        homekit_characteristic_notify(&ch_target_fan_state, ch_target_fan_state.value);
        homekit_characteristic_notify(&ch_rotation_speed, ch_rotation_speed.value);
}

static float mode_to_speed_snap(fan_mode_t mode) {
        switch (mode) {
        case FAN_MODE_LOW:  return 0.0f;
        case FAN_MODE_MED:  return 50.0f;
        case FAN_MODE_HIGH: return 100.0f;
        default:            return 0.0f;
        }
}

static fan_mode_t speed_to_mode(float speed) {
        if (speed <= SPEED_LOW_MAX) return FAN_MODE_LOW;
        if (speed <= SPEED_MED_MAX) return FAN_MODE_MED;
        return FAN_MODE_HIGH;
}

static void set_home_speed_from_mode_snap(void) {
        ch_rotation_speed.value = HOMEKIT_FLOAT(mode_to_speed_snap(fan_mode));
}

static void set_home_speed_exact(float speed_0_100) {
        if (speed_0_100 < 0.0f) speed_0_100 = 0.0f;
        if (speed_0_100 > 100.0f) speed_0_100 = 100.0f;
        ch_rotation_speed.value = HOMEKIT_FLOAT(speed_0_100);
}

// ----- Identify -----
static void accessory_identify_task(void *args) {
        for (int i = 0; i < 3; i++) {
                led_write(true);
                vTaskDelay(pdMS_TO_TICKS(100));
                led_write(false);
                vTaskDelay(pdMS_TO_TICKS(100));
                vTaskDelay(pdMS_TO_TICKS(250));
        }
        led_write(fan_mode == FAN_MODE_HIGH);
        vTaskDelete(NULL);
}

static void accessory_identify(homekit_value_t _value) {
        ESP_LOGI("INFORMATION", "Accessory identify");
        xTaskCreate(accessory_identify_task, "Accessory identify", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
}

// ----- FAN getters/setters -----
homekit_value_t fan_active_get(void) {
        return HOMEKIT_UINT8(active_u8);
}

void fan_active_set(homekit_value_t value) {
        if (value.format != homekit_format_uint8) {
                ESP_LOGE(TAG, "Invalid ACTIVE format: %d (expected uint8)", value.format);
                return;
        }

        uint8_t req = value.uint8_value ? 1 : 0;

        // Manual action => pause auto for a while (only meaningful when AUTO enabled)
        manual_hold_until_ms = now_ms() + MANUAL_HOLD_MS;

        if (req == 0) {
                ESP_LOGI(TAG, "Fan tile OFF -> level 1 (LOW), pulse LOW, speed=0");
                active_u8 = 0;
                remote_pulse_mode(FAN_MODE_LOW);
                set_home_speed_from_mode_snap();
                sync_and_notify_fan();
                return;
        }

        // Active ON: if currently LOW/OFF, jump to MED and set speed to 34%
        ESP_LOGI(TAG, "Fan tile ON");
        active_u8 = 1;

        if (fan_mode == FAN_MODE_LOW) {
                ESP_LOGI(TAG, "Active ON while LOW -> switch to MED and set speed=34%%");
                remote_pulse_mode(FAN_MODE_MED);
                set_home_speed_exact(SPEED_ON_JUMP_TO_MED);
        } else {
                ESP_LOGI(TAG, "Active ON -> pulse current mode");
                remote_pulse_mode(fan_mode);
                set_home_speed_from_mode_snap();
        }

        sync_and_notify_fan();
}

homekit_value_t current_fan_state_get(void) {
        return (active_u8 == 0) ? HOMEKIT_UINT8(0) : HOMEKIT_UINT8(2);
}

homekit_value_t target_fan_state_get(void) {
        return HOMEKIT_UINT8(auto_enabled ? 1 : 0); // 0=manual, 1=auto
}

void target_fan_state_set(homekit_value_t value) {
        if (value.format != homekit_format_uint8) {
                ESP_LOGE(TAG, "Invalid TARGET_FAN_STATE format: %d (expected uint8)", value.format);
                return;
        }

        bool new_auto = (value.uint8_value == 1);
        auto_enabled = new_auto;

        ch_target_fan_state.value = HOMEKIT_UINT8(auto_enabled ? 1 : 0);
        homekit_characteristic_notify(&ch_target_fan_state, ch_target_fan_state.value);

        // If switching to AUTO, allow auto control to take over immediately
        if (auto_enabled) {
                manual_hold_until_ms = 0;
                ESP_LOGI(TAG, "Target fan state set to AUTO");
        } else {
                ESP_LOGI(TAG, "Target fan state set to MANUAL");
        }
}

homekit_value_t rotation_speed_get(void) {
        return ch_rotation_speed.value;
}

void rotation_speed_set(homekit_value_t value) {
        if (value.format != homekit_format_float) {
                ESP_LOGE(TAG, "Invalid ROTATION_SPEED format: %d (expected float)", value.format);
                return;
        }

        float s = value.float_value;
        if (s < 0.0f) s = 0.0f;
        if (s > 100.0f) s = 100.0f;

        fan_mode_t requested = speed_to_mode(s);

        ESP_LOGI(TAG, "Slider set: %.1f%% -> mode %d", s, (int)requested);

        // Manual action => pause auto for a while
        manual_hold_until_ms = now_ms() + MANUAL_HOLD_MS;

        // LOW range => treat as OFF
        active_u8 = (requested == FAN_MODE_LOW) ? 0 : 1;

        // Pulse only if mode changes
        if (requested != fan_mode) {
                remote_pulse_mode(requested);
        }

        // Snap slider for clear UI (0/50/100)
        set_home_speed_from_mode_snap();

        sync_and_notify_fan();
}

// ----- Button handling (unchanged) -----
void button_callback(button_event_t event, void *context) {
        (void)context;
        switch (event) {
        case button_event_single_press:
                ESP_LOGI("BUTTON", "Single press");
                lifecycle_request_update_and_reboot();
                break;
        case button_event_double_press:
                ESP_LOGI("BUTTON", "Double press");
                homekit_server_reset();
                esp_restart();
                break;
        case button_event_long_press:
                ESP_LOGI("BUTTON", "Long press");
                lifecycle_factory_reset_and_reboot();
                break;
        default:
                ESP_LOGI("BUTTON", "Unknown button event: %d", event);
                break;
        }
}

// ----- HomeKit accessory definition: FAN2 + Sensors -----
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
        HOMEKIT_ACCESSORY(.id = 1, .category = homekit_accessory_category_fans, .services = (homekit_service_t*[]) {

                HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics = (homekit_characteristic_t*[]) {
                        &name,
                        &manufacturer,
                        &serial,
                        &model,
                        &revision,
                        HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify),
                        NULL
                }),

                HOMEKIT_SERVICE(FAN2, .primary = true, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Fan"),
                        &ch_fan_active,
                        &ch_current_fan_state,
                        &ch_target_fan_state,   // now used (AUTO by default)
                        &ch_rotation_speed,
                        NULL
                }),

                HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Bathroom Temperature"),
                        &ch_current_temperature,
                        NULL
                }),

                HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Bathroom Humidity"),
                        &ch_current_humidity,
                        NULL
                }),

                NULL
        }),
        NULL
};
#pragma GCC diagnostic pop

homekit_server_config_t config = {
        .accessories = accessories,
        .password = CONFIG_ESP_SETUP_CODE,
        .setupId = CONFIG_ESP_SETUP_ID,
};

// --- One-time LOW pulse after HomeKit is started ---
static void boot_low_pulse_task(void *arg) {
        (void)arg;

        vTaskDelay(pdMS_TO_TICKS(1200));

        if (!boot_low_pulsed) {
                boot_low_pulsed = true;
                ESP_LOGI(TAG, "Boot sync: pulsing LOW once (level 1 / OFF)");
                remote_pulse_mode(FAN_MODE_LOW);

                active_u8 = 0;
                set_home_speed_from_mode_snap();
                sync_and_notify_fan();
        }

        vTaskDelete(NULL);
}

// ----------------- HUMIDITY CONTROL TASK -----------------
static fan_mode_t compute_auto_mode(float rh, float rhbase, float rise_per_min) {
        float med_on  = fmaxf(RH_ABS_MED_ON,  rhbase + RH_BASE_PLUS_MED);
        float high_on = fmaxf(RH_ABS_HIGH_ON, rhbase + RH_BASE_PLUS_HIGH);

        if (rise_per_min >= RH_RISE_HIGH_PER_MIN) return FAN_MODE_HIGH;
        if (rise_per_min >= RH_RISE_MED_PER_MIN && rh >= (med_on - 1.0f)) return FAN_MODE_MED;

        if (rh >= high_on) return FAN_MODE_HIGH;
        if (rh >= med_on) return FAN_MODE_MED;
        return FAN_MODE_LOW;
}

static void apply_mode_with_min_runtime(fan_mode_t desired) {
        int64_t t = now_ms();

        // AUTO must be enabled
        if (!auto_enabled) return;

        // Manual override window
        if (t < manual_hold_until_ms) return;

        if (desired == fan_mode) return;

        if (fan_mode == FAN_MODE_HIGH && desired != FAN_MODE_HIGH) {
                if ((t - mode_enter_ms) < HIGH_MIN_RUN_MS) return;
        }
        if (fan_mode == FAN_MODE_MED && desired == FAN_MODE_LOW) {
                if ((t - mode_enter_ms) < MED_MIN_RUN_MS) return;
        }

        ESP_LOGI(TAG, "AUTO: switching %d -> %d", (int)fan_mode, (int)desired);

        active_u8 = (desired == FAN_MODE_LOW) ? 0 : 1;

        remote_pulse_mode(desired);
        mode_enter_ms = t;

        set_home_speed_from_mode_snap();
        sync_and_notify_fan();
}

static void humidity_task(void *arg) {
        (void)arg;

        last_sample_ms = now_ms();
        mode_enter_ms  = last_sample_ms;

        // For reduced HomeKit notify spam
        float last_notified_temp = NAN;
        float last_notified_rh   = NAN;

        for (;;) {
                vTaskDelay(pdMS_TO_TICKS(HUM_SAMPLE_MS));

                float temp = 0.0f, hum = 0.0f;
                esp_err_t err = sht3x_read_temperature_humidity(g_sht3x_addr, &temp, &hum);
                if (err != ESP_OK) {
                        ESP_LOGW(TAG, "SHT3x read failed: %s", esp_err_to_name(err));
                        continue;
                }

                int64_t t_ms = now_ms();
                float dt_min = (float)(t_ms - last_sample_ms) / 60000.0f;
                if (dt_min <= 0.0001f) dt_min = 0.0001f;

                float rise_per_min = 0.0f;
                if (!isnan(last_rh)) rise_per_min = (hum - last_rh) / dt_min;

                rh_now = hum;
                t_now  = temp;

                // ---- Publish sensor data to HomeKit (with small thresholds) ----
                bool notify_temp = isnan(last_notified_temp) || fabsf(temp - last_notified_temp) >= TEMP_NOTIFY_DELTA_C;
                bool notify_rh   = isnan(last_notified_rh)   || fabsf(hum  - last_notified_rh)   >= RH_NOTIFY_DELTA_PCT;

                if (notify_temp) {
                        ch_current_temperature.value = HOMEKIT_FLOAT(temp);
                        homekit_characteristic_notify(&ch_current_temperature, ch_current_temperature.value);
                        last_notified_temp = temp;
                }
                if (notify_rh) {
                        ch_current_humidity.value = HOMEKIT_FLOAT(hum);
                        homekit_characteristic_notify(&ch_current_humidity, ch_current_humidity.value);
                        last_notified_rh = hum;
                }

                // ---- Baseline learning only when AUTO is allowed to run ----
                if (auto_enabled && t_ms >= manual_hold_until_ms) {
                        bool calm = fabsf(rise_per_min) < 0.5f;
                        if (fan_mode == FAN_MODE_LOW && calm) {
                                if (!rh_base_valid) {
                                        rh_base = hum;
                                        rh_base_valid = true;
                                } else {
                                        rh_base = rh_base + BASELINE_ALPHA * (hum - rh_base);
                                }
                        }
                }

                float base = rh_base_valid ? rh_base : hum;

                fan_mode_t desired = compute_auto_mode(hum, base, rise_per_min);

                float med_on  = fmaxf(RH_ABS_MED_ON,  base + RH_BASE_PLUS_MED);
                float high_on = fmaxf(RH_ABS_HIGH_ON, base + RH_BASE_PLUS_HIGH);

                // Hysteresis for stepping down
                if (fan_mode == FAN_MODE_HIGH && desired != FAN_MODE_HIGH) {
                        if (hum > (high_on - RH_HIGH_OFF_HYST)) desired = FAN_MODE_HIGH;
                }
                if (fan_mode == FAN_MODE_MED && desired == FAN_MODE_LOW) {
                        if (hum > (med_on - RH_MED_OFF_HYST)) desired = FAN_MODE_MED;
                }

                ESP_LOGI(TAG,
                         "SHT3x: T=%.2fC RH=%.1f%% base=%.1f rise=%.2f%%/min mode=%d desired=%d hold=%s auto=%s",
                         temp, hum, base, rise_per_min, (int)fan_mode, (int)desired,
                         (t_ms < manual_hold_until_ms) ? "YES" : "no",
                         auto_enabled ? "YES" : "no");

                apply_mode_with_min_runtime(desired);

                last_rh = hum;
                last_sample_ms = t_ms;
        }
}
// ---------------------------------------------------------

void on_wifi_ready() {
        static bool homekit_started = false;

        if (homekit_started) {
                ESP_LOGI("INFORMATION", "HomeKit server already running; skipping re-initialization");
                return;
        }

        ESP_LOGI("INFORMATION", "Starting HomeKit server...");
        homekit_server_init(&config);
        homekit_started = true;

        xTaskCreate(boot_low_pulse_task, "boot_low_pulse", 2048, NULL, 2, NULL);
}

void app_main(void) {
        ESP_ERROR_CHECK(lifecycle_nvs_init());
        lifecycle_log_post_reset_state("INFORMATION");
        ESP_ERROR_CHECK(lifecycle_configure_homekit(&revision, &ota_trigger, "INFORMATION"));

        gpio_init_all();

        // --- SHT3x init ---
        g_sht3x_addr = (uint8_t)CONFIG_SHT3X_I2C_ADDRESS;
        i2c_master_init();
        ESP_ERROR_CHECK(sht3x_init(g_sht3x_addr));

#if CONFIG_ESP_HUMIDITY_AUTO_ENABLE
        xTaskCreate(humidity_task, "humidity_task", 4096, NULL, 3, NULL);
        ESP_LOGI(TAG, "Humidity auto-control: ENABLED");
#else
        ESP_LOGW(TAG, "Humidity auto-control: DISABLED (menuconfig)");
#endif

        // Bind getters/setters
        ch_fan_active.getter = fan_active_get;
        ch_fan_active.setter = fan_active_set;

        ch_current_fan_state.getter = current_fan_state_get;

        ch_target_fan_state.getter = target_fan_state_get;
        ch_target_fan_state.setter = target_fan_state_set;

        ch_rotation_speed.getter = rotation_speed_get;
        ch_rotation_speed.setter = rotation_speed_set;

        button_config_t btn_cfg = button_config_default(button_active_low);
        btn_cfg.max_repeat_presses = 3;
        btn_cfg.long_press_time = 1000;

        if (button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL)) {
                ESP_LOGE("BUTTON", "Failed to initialize button");
        }

        esp_err_t wifi_err = wifi_start(on_wifi_ready);
        if (wifi_err != ESP_OK) {
                ESP_LOGE("WIFI", "Failed to start WiFi: %s", esp_err_to_name(wifi_err));
        }
}
