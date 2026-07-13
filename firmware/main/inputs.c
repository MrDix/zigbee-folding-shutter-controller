#include "inputs.h"

#include "board.h"
#include "shutter.h"
#include "zb_app.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

static const char *TAG = "inputs";

#define SCAN_MS          10
#define DEBOUNCE_SCANS   3
#define LEARN_SHORT_MS   1500
#define LEARN_RESET_MS   5000

static contact_event_cb_t s_contact_cb;
static uint16_t s_stable;        /* debounced raw shift-register word */
static uint16_t s_last_raw;
static uint8_t s_same_count;

/* All inputs idle high (pull-up), active low. */
static inline bool bit_active(uint16_t w, int bit) { return ((w >> bit) & 1) == 0; }

static uint16_t hc165_read(void)
{
    /* latch the parallel inputs */
    gpio_set_level(PIN_SR_PL, 0);
    ets_delay_us(2);
    gpio_set_level(PIN_SR_PL, 1);
    ets_delay_us(2);

    uint16_t v = 0;
    for (int i = 0; i < 16; i++) {
        v = (v << 1) | (gpio_get_level(PIN_SR_Q7) & 1);
        gpio_set_level(PIN_SR_CP, 1);
        ets_delay_us(1);
        gpio_set_level(PIN_SR_CP, 0);
        ets_delay_us(1);
    }
    return v;
}

static void emit_contact(uint8_t idx)
{
    static const int contact_bits[4] = { SR_BIT_CONTACT1, SR_BIT_CONTACT2,
                                         SR_BIT_CONTACT3, SR_BIT_CONTACT4 };
    static const int tamper_bits[4]  = { SR_BIT_TAMPER1, SR_BIT_TAMPER2,
                                         SR_BIT_TAMPER3, SR_BIT_TAMPER4 };
    if (s_contact_cb) {
        contact_event_t e = {
            .index = idx,
            .closed = bit_active(s_stable, contact_bits[idx]),
            .tamper = !bit_active(s_stable, tamper_bits[idx]), /* loop open = tamper */
        };
        s_contact_cb(&e);
    }
}

static void handle_change(uint16_t old, uint16_t now)
{
    static const int contact_bits[4] = { SR_BIT_CONTACT1, SR_BIT_CONTACT2,
                                         SR_BIT_CONTACT3, SR_BIT_CONTACT4 };
    static const int tamper_bits[4]  = { SR_BIT_TAMPER1, SR_BIT_TAMPER2,
                                         SR_BIT_TAMPER3, SR_BIT_TAMPER4 };
    uint16_t diff = old ^ now;

    for (int i = 0; i < 4; i++) {
        if ((diff >> contact_bits[i]) & 1 || (diff >> tamper_bits[i]) & 1) {
            emit_contact(i);
        }
    }

    /* wall buttons: press = falling edge; press while moving = stop */
    if ((diff >> SR_BIT_BTN_OPEN) & 1 && bit_active(now, SR_BIT_BTN_OPEN)) {
        if (shutter_get_state() != SHUTTER_IDLE) {
            ESP_LOGI(TAG, "wall button: stop");
            shutter_stop();
        } else {
            ESP_LOGI(TAG, "wall button: open");
            shutter_open();
        }
    }
    if ((diff >> SR_BIT_BTN_CLOSE) & 1 && bit_active(now, SR_BIT_BTN_CLOSE)) {
        if (shutter_get_state() != SHUTTER_IDLE) {
            ESP_LOGI(TAG, "wall button: stop");
            shutter_stop();
        } else {
            ESP_LOGI(TAG, "wall button: close");
            shutter_close();
        }
    }
}

static void learn_button_poll(void)
{
    static uint32_t held_ms;
    static bool reset_fired;

    if (gpio_get_level(PIN_BTN_LEARN) == 0) {
        held_ms += SCAN_MS;
        if (held_ms >= LEARN_RESET_MS && !reset_fired) {
            reset_fired = true;
            ESP_LOGW(TAG, "setup button held %d ms -> factory reset / re-pair",
                     LEARN_RESET_MS);
            zb_app_factory_reset();
        }
    } else {
        if (held_ms >= SCAN_MS * DEBOUNCE_SCANS && held_ms < LEARN_SHORT_MS &&
            !reset_fired) {
            ESP_LOGI(TAG, "setup button short press -> learn cycle");
            shutter_start_learn();
        }
        held_ms = 0;
        reset_fired = false;
    }
}

static void inputs_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SCAN_MS));

        uint16_t raw = hc165_read();
        if (raw == s_last_raw) {
            if (s_same_count < DEBOUNCE_SCANS) {
                s_same_count++;
            } else if (raw != s_stable) {
                uint16_t old = s_stable;
                s_stable = raw;
                handle_change(old, raw);
            }
        } else {
            s_last_raw = raw;
            s_same_count = 0;
        }

        learn_button_poll();
    }
}

void inputs_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_SR_PL) | (1ULL << PIN_SR_CP),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_SR_PL, 1);
    gpio_set_level(PIN_SR_CP, 0);

    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_SR_Q7),
        .mode = GPIO_MODE_INPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << PIN_BTN_LEARN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&btn));

    /* settle + take the initial state without emitting events */
    vTaskDelay(pdMS_TO_TICKS(20));
    s_stable = s_last_raw = hc165_read();
    s_same_count = DEBOUNCE_SCANS;

    xTaskCreate(inputs_task, "inputs", 3072, NULL, 5, NULL);
}

void inputs_register_contact_cb(contact_event_cb_t cb) { s_contact_cb = cb; }

bool inputs_contact_closed(uint8_t index)
{
    static const int bits[4] = { SR_BIT_CONTACT1, SR_BIT_CONTACT2,
                                 SR_BIT_CONTACT3, SR_BIT_CONTACT4 };
    return (index < 4) ? bit_active(s_stable, bits[index]) : false;
}

bool inputs_tamper(uint8_t index)
{
    static const int bits[4] = { SR_BIT_TAMPER1, SR_BIT_TAMPER2,
                                 SR_BIT_TAMPER3, SR_BIT_TAMPER4 };
    return (index < 4) ? !bit_active(s_stable, bits[index]) : false;
}
