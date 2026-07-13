/*
 * Monitored inputs: 4 window contacts + 4 tamper loops (via the 74HC165
 * chain), the two wall buttons, and the local learn/setup button.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t index;      /* 0..3 */
    bool closed;        /* reed contact closed (window closed) */
    bool tamper;        /* tamper/continuity loop broken */
} contact_event_t;

typedef void (*contact_event_cb_t)(const contact_event_t *evt);

void inputs_init(void);
void inputs_register_contact_cb(contact_event_cb_t cb);

/* current debounced state, for initial attribute values */
bool inputs_contact_closed(uint8_t index);
bool inputs_tamper(uint8_t index);
