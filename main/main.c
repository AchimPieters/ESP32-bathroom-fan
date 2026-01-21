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
 **/

#include <stdio.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#include "esp32-lcm.h"
#include <button.h>

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

// ----- State -----
typedef enum {
        FAN_MODE_LOW  = 0,// stand 1 = laag = "uit"
        FAN_MODE_MED  = 1,// stand 2 = middel
        FAN_MODE_HIGH = 2 // stand 3 = hoog
} fan_mode_t;

static fan_mode_t fan_mode = FAN_MODE_LOW;

// Fan tile should be OFF after reboot (but physically fan is LOW)
static uint8_t active_u8 = 0;

static int64_t last_pulse_ms = 0;
static bool boot_low_pulsed = false;

// HomeKit characteristics
#define DEVICE_NAME "Badkamer Ventilator"
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

// ---- FAN2 characteristics (globals, so we can attach getters/setters later) ----
homekit_characteristic_t ch_fan_active        = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);            // uint8 0/1
homekit_characteristic_t ch_current_fan_state = HOMEKIT_CHARACTERISTIC_(CURRENT_FAN_STATE, 0); // uint8 enum
homekit_characteristic_t ch_target_fan_state  = HOMEKIT_CHARACTERISTIC_(TARGET_FAN_STATE, 0);  // keep manual

// ---- 2 speed switches (globals; attach getters/setters later) ----
homekit_characteristic_t ch_med_on  = HOMEKIT_CHARACTERISTIC_(ON, false);
homekit_characteristic_t ch_high_on = HOMEKIT_CHARACTERISTIC_(ON, false);

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
                ESP_LOGI(TAG, "Remote pulse: LOW (stand 1 / OFF-default)");
                break;
        case FAN_MODE_MED:
                press_pulse(FAN_MED_GPIO, PULSE_MS);
                ESP_LOGI(TAG, "Remote pulse: MED (stand 2)");
                break;
        case FAN_MODE_HIGH:
                press_pulse(FAN_HIGH_GPIO, PULSE_MS);
                ESP_LOGI(TAG, "Remote pulse: HIGH (stand 3)");
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

static void set_switch_states_from_mode(void) {
        // Only MED or HIGH can be "on"; LOW/OFF => both off
        ch_med_on.value  = HOMEKIT_BOOL(fan_mode == FAN_MODE_MED);
        ch_high_on.value = HOMEKIT_BOOL(fan_mode == FAN_MODE_HIGH);
}

static void notify_switches(void) {
        homekit_characteristic_notify(&ch_med_on,  ch_med_on.value);
        homekit_characteristic_notify(&ch_high_on, ch_high_on.value);
}

/*
   CurrentFanState: 0=Inactive, 2=Blowing Air
 */
static void sync_and_notify_fan(void) {
        ch_fan_active.value = HOMEKIT_UINT8(active_u8);

        if (active_u8 == 0) {
                ch_current_fan_state.value = HOMEKIT_UINT8(0); // Home shows OFF
        } else {
                ch_current_fan_state.value = HOMEKIT_UINT8(2); // blowing
        }

        ch_target_fan_state.value = HOMEKIT_UINT8(0); // manual

        homekit_characteristic_notify(&ch_fan_active, ch_fan_active.value);
        homekit_characteristic_notify(&ch_current_fan_state, ch_current_fan_state.value);
        homekit_characteristic_notify(&ch_target_fan_state, ch_target_fan_state.value);
}

static void sync_and_notify_all(void) {
        sync_and_notify_fan();
        set_switch_states_from_mode();
        notify_switches();
}

static void gpio_init_all(void) {
        gpio_reset_pin(LED_GPIO);
        gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
        led_write(false);

        init_remote_button_gpio(FAN_LOW_GPIO);
        init_remote_button_gpio(FAN_MED_GPIO);
        init_remote_button_gpio(FAN_HIGH_GPIO);

        // Default: stand 1 (LOW) but Home shows OFF
        fan_mode = FAN_MODE_LOW;
        active_u8 = 0;

        ch_fan_active.value = HOMEKIT_UINT8(active_u8);
        ch_current_fan_state.value = HOMEKIT_UINT8(0);
        ch_target_fan_state.value = HOMEKIT_UINT8(0);

        set_switch_states_from_mode();
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

// ----- FAN2 getters/setters -----
homekit_value_t fan_active_get(void) {
        return HOMEKIT_UINT8(active_u8);
}

void fan_active_set(homekit_value_t value) {
        if (value.format != homekit_format_uint8) {
                ESP_LOGE(TAG, "Invalid ACTIVE format: %d (expected uint8)", value.format);
                return;
        }

        uint8_t req = value.uint8_value ? 1 : 0;

        if (req == 0) {
                // OFF => stand 1 (LOW) + pulse LOW
                ESP_LOGI(TAG, "Fan tile OFF -> stand 1 (LOW), pulse LOW");
                active_u8 = 0;
                remote_pulse_mode(FAN_MODE_LOW);
                sync_and_notify_all();
                return;
        }

        // ON => keep current mode, pulse it to re-sync
        ESP_LOGI(TAG, "Fan tile ON -> pulse current mode");
        active_u8 = 1;
        remote_pulse_mode(fan_mode);
        sync_and_notify_all();
}

homekit_value_t current_fan_state_get(void) {
        return (active_u8 == 0) ? HOMEKIT_UINT8(0) : HOMEKIT_UINT8(2);
}

homekit_value_t target_fan_state_get(void) {
        return HOMEKIT_UINT8(0); // manual
}

void target_fan_state_set(homekit_value_t value) {
        // Accept but ignore; keep manual
        (void)value;
        ch_target_fan_state.value = HOMEKIT_UINT8(0);
        homekit_characteristic_notify(&ch_target_fan_state, ch_target_fan_state.value);
}

// ----- Switch getters/setters -----
homekit_value_t med_get(void)  {
        return HOMEKIT_BOOL(fan_mode == FAN_MODE_MED);
}
homekit_value_t high_get(void) {
        return HOMEKIT_BOOL(fan_mode == FAN_MODE_HIGH);
}

static void select_mode_and_pulse(fan_mode_t requested_mode) {
        // Selecting a speed implies ON in Home
        active_u8 = 1;

        // Always pulse when pressing switch
        remote_pulse_mode(requested_mode);

        sync_and_notify_all();
}

void med_set(homekit_value_t value) {
        if (value.format != homekit_format_bool) return;

        if (value.bool_value) {
                ESP_LOGI(TAG, "Switch: Snelheid: Middel -> pulse MED");
                select_mode_and_pulse(FAN_MODE_MED);
        } else {
                // MED off => back to LOW and Home OFF
                ESP_LOGI(TAG, "Switch: Snelheid: Middel OFF -> stand 1 (LOW), Home OFF, pulse LOW");
                active_u8 = 0;
                remote_pulse_mode(FAN_MODE_LOW);
                sync_and_notify_all();
        }
}

void high_set(homekit_value_t value) {
        if (value.format != homekit_format_bool) return;

        if (value.bool_value) {
                ESP_LOGI(TAG, "Switch: Snelheid: Hoog -> pulse HIGH");
                select_mode_and_pulse(FAN_MODE_HIGH);
        } else {
                // HIGH off => back to LOW and Home OFF
                ESP_LOGI(TAG, "Switch: Snelheid: Hoog OFF -> stand 1 (LOW), Home OFF, pulse LOW");
                active_u8 = 0;
                remote_pulse_mode(FAN_MODE_LOW);
                sync_and_notify_all();
        }
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

// ----- HomeKit accessory definition: FAN2 + 2 SWITCH services -----
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

                // Primary fan tile (OFF = stand 1)
                HOMEKIT_SERVICE(FAN2, .primary = true, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Ventilator"),
                        &ch_fan_active,
                        &ch_current_fan_state,
                        &ch_target_fan_state,
                        NULL
                }),

                // Two speed switches
                HOMEKIT_SERVICE(SWITCH, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Snelheid: Middel"),
                        &ch_med_on,
                        NULL
                }),

                HOMEKIT_SERVICE(SWITCH, .characteristics = (homekit_characteristic_t*[]) {
                        HOMEKIT_CHARACTERISTIC(NAME, "Snelheid: Hoog"),
                        &ch_high_on,
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
                ESP_LOGI(TAG, "Boot sync: pulsing LOW once (stand 1 / OFF)");
                remote_pulse_mode(FAN_MODE_LOW);

                // Keep Home state OFF
                active_u8 = 0;
                sync_and_notify_all();
        }

        vTaskDelete(NULL);
}

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

        // Bind getters/setters (THIS avoids the macro initializer issue)
        ch_fan_active.getter = fan_active_get;
        ch_fan_active.setter = fan_active_set;

        ch_current_fan_state.getter = current_fan_state_get;

        ch_target_fan_state.getter = target_fan_state_get;
        ch_target_fan_state.setter = target_fan_state_set;

        ch_med_on.getter = med_get;
        ch_med_on.setter = med_set;

        ch_high_on.getter = high_get;
        ch_high_on.setter = high_set;

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
