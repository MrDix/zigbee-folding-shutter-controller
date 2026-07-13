/*
 * Shutter pair control: two 24 V DC wing motors driven as one folding shutter.
 *
 * Replicates the behaviour of the original control unit:
 *  - covering-wing sequencing (wing 1 opens first, closes last; fixed stagger)
 *  - end of travel detected by motor-current rise (no limit switches)
 *  - obstacle mid-travel: reverse the pair back to where the move started
 *  - overcurrent right after start (frozen shutter): treated as end of travel
 *  - PWM soft start / soft stop
 *  - learn cycle: measures travel time and normal running current per motor
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SHUTTER_IDLE,
    SHUTTER_MOVING,      /* regular commanded move */
    SHUTTER_REVERSING,   /* obstacle recovery move */
    SHUTTER_LEARNING,    /* calibration cycle */
} shutter_state_t;

/* pct_closed: 0 = fully open .. 100 = fully closed (ZCL lift convention) */
typedef void (*shutter_event_cb_t)(uint8_t pct_closed, bool moving);

void shutter_init(void);
void shutter_register_cb(shutter_event_cb_t cb);

void shutter_open(void);
void shutter_close(void);
void shutter_stop(void);
void shutter_goto_percent(uint8_t pct_closed);
void shutter_start_learn(void);

uint8_t shutter_position(void);          /* % closed */
shutter_state_t shutter_get_state(void);
bool shutter_is_calibrated(void);
