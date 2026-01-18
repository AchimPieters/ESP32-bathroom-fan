/**
   Copyright 2025 Achim Pieters | StudioPieters®

   MIT License (unchanged)
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

// Optional status LED
#define LED_GPIO CONFIG_ESP_LED_GPIO
static const char *TAG = "FAN_REMOTE";

// Remote "button" outputs (each gives one pulse)
#define FAN_LOW_GPIO  CONFIG_ESP_FAN_LOW_GPIO
#define FAN_MED_GPIO  CONFIG_ESP_FAN_MED_GPIO
#define FAN_HIGH_GPIO CONFIG_ESP_FAN_HIGH_GPIO

// Pulse timing
#define PULSE_MS   CONFIG_ESP_REMOTE_PULSE_MS
#define LOCKOUT_MS CONFIG_ESP_REMOTE_LOCKOUT_MS

// ----- State -----
typedef enum {
    FAN_MODE_LOW  = 1,
    FAN_MODE_MED  = 2,
    FAN_MODE_HIGH = 3
} fan_mode_t;

static fan_mode_t fan_mode = FAN_MODE_LOW;
static int rotation_speed_pct = 33; // HomeKit slider value
static int64_t last_pulse_ms = 0;

// HomeKit characteristics
#define DEVICE_NAME "Badkamer Ventilator"
#define DEVICE_MANUFACTURER "StudioPieters®"
#define DEVICE_SERIAL "NL-VENT-0001"
#define DEVICE_MODEL "FAN-3MODE-REMOTE"

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, DEVICE_NAME);
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  DEVICE_MANUFACTURER);
homekit_characteristic_t serial = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, DEVICE_SERIAL);
homekit_characteristic_t model = HOMEKIT_CHARACTERISTIC_(MODEL, DEVICE_MODEL);
homekit_characteristic_t revision = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION, LIFECYCLE_DEFAULT_FW_VERSION);
homekit_characteristic_t ota_trigger = API_OTA_TRIGGER;

// ----- Helpers -----
static void led_write(bool on) {
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

static int64_t now_ms(void) {
    return esp_timer_get_time() / 1000;
}

// Open-drain "press" = pull to GND (0), "release" = Hi-Z (1)
static void press_pulse(gpio_num_t pin, int pulse_ms) {
    int64_t t = now_ms();

    if (LOCKOUT_MS > 0 && (t - last_pulse_ms < LOCKOUT_MS)) {
        ESP_LOGW(TAG, "Pulse ignored (lockout)");
        return;
    }
    last_pulse_ms = t;

    ESP_LOGI(TAG, "Pulse GPIO %d for %dms (active-low, open-drain)", (int)pin, pulse_ms);

    // press
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(pulse_ms));
    // release (Hi-Z)
    gpio_set_level(pin, 1);
}

static void init_remote_button_gpio(gpio_num_t pin) {
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT_OD); // open-drain output
    gpio_set_level(pin, 1); // released (Hi-Z)
}

static void remote_set_mode(fan_mode_t mode) {
    switch (mode) {
        case FAN_MODE_LOW:
            press_pulse(FAN_LOW_GPIO, PULSE_MS);
            ESP_LOGI(TAG, "Requested remote mode: LOW");
            break;
        case FAN_MODE_MED:
            press_pulse(FAN_MED_GPIO, PULSE_MS);
            ESP_LOGI(TAG, "Requested remote mode: MED");
            break;
        case FAN_MODE_HIGH:
            press_pulse(FAN_HIGH_GPIO, PULSE_MS);
            ESP_LOGI(TAG, "Requested remote mode: HIGH");
            break;
        default:
            press_pulse(FAN_LOW_GPIO, PULSE_MS);
            ESP_LOGW(TAG, "Unknown mode -> pulse LOW");
            mode = FAN_MODE_LOW;
            break;
    }

    fan_mode = mode;

    // Optional LED feedback: ON only in HIGH
    led_write(fan_mode == FAN_MODE_HIGH);
}

static void gpio_init_all(void) {
    // LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    led_write(false);

    // Remote button GPIOs (open-drain)
    init_remote_button_gpio(FAN_LOW_GPIO);
    init_remote_button_gpio(FAN_MED_GPIO);
    init_remote_button_gpio(FAN_HIGH_GPIO);

    // Note: we do NOT auto-pulse on boot
    fan_mode = FAN_MODE_LOW;
    rotation_speed_pct = 33;
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

// ----- HomeKit FAN2 behavior -----

// Per jouw wens: stand 1 = laag (standaard). We laten HomeKit "OFF" niet echt uitzetten.
static bool fan_active = true;

homekit_value_t fan_active_get(void) {
    return HOMEKIT_BOOL(true);
}

void fan_active_set(homekit_value_t value) {
    if (value.format != homekit_format_bool) {
        ESP_LOGE("ERROR", "Invalid value format: %d", value.format);
        return;
    }

    if (!value.bool_value) {
        // HomeKit OFF -> we interpreteren dit als "LOW" (default)
        ESP_LOGI(TAG, "HomeKit OFF -> pulsing LOW (default)");
        rotation_speed_pct = 33;
        remote_set_mode(FAN_MODE_LOW);

        // Keep HomeKit ON (since it cannot be truly off by requirement)
        fan_active = true;
        return;
    }

    fan_active = true;
}

homekit_value_t rotation_speed_get(void) {
    return HOMEKIT_INT(rotation_speed_pct);
}

static fan_mode_t mode_from_pct(int pct) {
    // 1..33 = LOW, 34..66 = MED, 67..100 = HIGH
    if (pct <= 33) return FAN_MODE_LOW;
    if (pct <= 66) return FAN_MODE_MED;
    return FAN_MODE_HIGH;
}

void rotation_speed_set(homekit_value_t value) {
    if (value.format != homekit_format_int) {
        ESP_LOGE("ERROR", "Invalid rotation_speed format: %d", value.format);
        return;
    }

    int pct = value.int_value;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;

    // 0 behandelen we als LOW (want altijd minimaal stand 1)
    if (pct == 0) pct = 1;

    rotation_speed_pct = pct;

    fan_mode_t requested = mode_from_pct(rotation_speed_pct);

    // Alleen pulsen als de “mode” echt verandert
    if (requested != fan_mode) {
        remote_set_mode(requested);
    } else {
        ESP_LOGI(TAG, "RotationSpeed change within same mode -> no pulse");
    }
}

// ----- Button handling (unchanged) -----
void button_callback(button_event_t event, void *context) {
    switch (event) {
    case button_event_single_press:
        ESP_LOGI("BUTTON", "Single press");
        lifecycle_request_update_and_reboot();
        break;
    case button_event_double_press:
        ESP_LOGI("BUTTON", "Double press");
        homekit_server_reset(); // Reset HomeKit
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

// ----- HomeKit accessory definition -----
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverride-init"
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id = 1, .category = homekit_accessory_category_fan, .services = (homekit_service_t*[]) {
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
            HOMEKIT_CHARACTERISTIC(NAME, "Ventilator"),
            HOMEKIT_CHARACTERISTIC(ON, true, .getter = fan_active_get, .setter = fan_active_set),
            HOMEKIT_CHARACTERISTIC(ROTATION_SPEED, 33, .getter = rotation_speed_get, .setter = rotation_speed_set),
            &ota_trigger,
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

void on_wifi_ready() {
    static bool homekit_started = false;

    if (homekit_started) {
        ESP_LOGI("INFORMATION", "HomeKit server already running; skipping re-initialization");
        return;
    }

    ESP_LOGI("INFORMATION", "Starting HomeKit server...");
    homekit_server_init(&config);
    homekit_started = true;
}

void app_main(void) {
    ESP_ERROR_CHECK(lifecycle_nvs_init());
    lifecycle_log_post_reset_state("INFORMATION");
    ESP_ERROR_CHECK(lifecycle_configure_homekit(&revision, &ota_trigger, "INFORMATION"));

    gpio_init_all();

    button_config_t btn_cfg = button_config_default(button_active_low);
    btn_cfg.max_repeat_presses = 3;
    btn_cfg.long_press_time = 1000;

    if (button_create(BUTTON_GPIO, btn_cfg, button_callback, NULL)) {
        ESP_LOGE("BUTTON", "Failed to initialize button");
    }

    esp_err_t wifi_err = wifi_start(on_wifi_ready);
    if (wifi_err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW("WIFI", "WiFi configuration not found; provisioning required");
    } else if (wifi_err != ESP_OK) {
        ESP_LOGE("WIFI", "Failed to start WiFi: %s", esp_err_to_name(wifi_err));
    }
}
