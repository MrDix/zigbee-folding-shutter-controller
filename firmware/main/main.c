/*
 * ShutterNode — Zigbee controller for 24 V DC folding-shutter window drives.
 *
 * Hardware: ESP32-C6 logic board + motor-driver power board (see the
 * hardware/ directory of this repository).
 */
#include "board.h"
#include "inputs.h"
#include "sense.h"
#include "shutter.h"
#include "zb_app.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG = "main";

#define VIN_REPORT_MS  30000

static void on_shutter_event(uint8_t pct_closed, bool moving)
{
    zb_app_update_position(pct_closed);
}

static void on_contact_event(const contact_event_t *evt)
{
    ESP_LOGI(TAG, "contact %d: %s%s", evt->index + 1,
             evt->closed ? "closed" : "open", evt->tamper ? " TAMPER" : "");
    zb_app_update_contact(evt->index, evt->closed, evt->tamper);
}

/*
 * Housekeeping: status LED + periodic supply-voltage report.
 * LED: fast blink = not joined / pairing, slow pulse = learn cycle,
 *      on = moving, off = idle.
 */
static void housekeeping_task(void *arg)
{
    uint32_t ms = 0, vin_ms = VIN_REPORT_MS;
    bool led = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ms += 100;
        vin_ms += 100;

        shutter_state_t st = shutter_get_state();
        if (!zb_app_joined()) {
            led = (ms / 200) % 2;           /* fast blink */
        } else if (st == SHUTTER_LEARNING) {
            led = (ms / 500) % 2;           /* slow blink */
        } else {
            led = (st != SHUTTER_IDLE);     /* on while moving */
        }
        gpio_set_level(PIN_LED_STATUS, led);

        if (vin_ms >= VIN_REPORT_MS) {
            vin_ms = 0;
            int mv = sense_read_vin_mv();
            if (mv >= 0) {
                zb_app_update_vin(mv);
            }
        }
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    gpio_config_t led = {
        .pin_bit_mask = 1ULL << PIN_LED_STATUS,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led));

    sense_init();
    shutter_init();
    shutter_register_cb(on_shutter_event);
    inputs_init();
    inputs_register_contact_cb(on_contact_event);
    zb_app_start();

    xTaskCreate(housekeeping_task, "housekeep", 3072, NULL, 3, NULL);

    ESP_LOGI(TAG, "ShutterNode up, calibrated=%d", shutter_is_calibrated());
}
