/*
 * ShutterNode logic board rev 1.0 — pin map and hardware constants.
 */
#pragma once

#include "driver/gpio.h"
#include "hal/adc_types.h"

/* ---- Motor driver 1: covering wing (opens first, closes last) ---- */
#define PIN_M1_INA      GPIO_NUM_4
#define PIN_M1_INB      GPIO_NUM_5
#define PIN_M1_PWM      GPIO_NUM_6
#define PIN_M1_SEL0     GPIO_NUM_7
#define ADC_CH_M1_CS    ADC_CHANNEL_0   /* GPIO0 */

/* ---- Motor driver 2: covered wing ---- */
#define PIN_M2_INA      GPIO_NUM_14
#define PIN_M2_INB      GPIO_NUM_15
#define PIN_M2_PWM      GPIO_NUM_18
#define PIN_M2_SEL0     GPIO_NUM_19
#define ADC_CH_M2_CS    ADC_CHANNEL_1   /* GPIO1 */

/*
 * MultiSense select: SEL0 level that routes the OUTA high-side current to
 * the CS pin. When driving OUTB, the inverted level is applied.
 * If CS reads near 0 mV while a motor is running under load, flip this.
 */
#define SEL0_LEVEL_FOR_OUTA  1

/* Current-sense scaling: CS pin sources Iout/K into R_sense. */
#define CS_RSENSE_OHM   680
#define CS_K_TYP        1540    /* Iout/Isense at 3.5..5.5 A, +/-15 % part spread */

/* ---- Supply-rail divider 100k/10k ---- */
#define ADC_CH_VIN      ADC_CHANNEL_2   /* GPIO2 */
#define VIN_DIV_NUM     11              /* Vin = Vadc * 11 */

/* ---- Input shift register chain: 2x 74HC165 ---- */
#define PIN_SR_PL       GPIO_NUM_20     /* parallel load, active low */
#define PIN_SR_CP       GPIO_NUM_21     /* shift clock, rising edge */
#define PIN_SR_Q7       GPIO_NUM_22     /* serial data out of the chain */

/*
 * Bit order as shifted out (first bit received = bit 15).
 * Stage 1 (closest to the MCU): D7..D4 = TAMPER4..1, D3..D0 = CONTACT4..1
 * Stage 2 (cascaded):           D7..D2 = tied low,   D1 = BTN_CLOSE, D0 = BTN_OPEN
 * All inputs are pulled up and switch to GND: closed contact / pressed button = 0.
 */
#define SR_BIT_TAMPER4   15
#define SR_BIT_TAMPER3   14
#define SR_BIT_TAMPER2   13
#define SR_BIT_TAMPER1   12
#define SR_BIT_CONTACT4  11
#define SR_BIT_CONTACT3  10
#define SR_BIT_CONTACT2  9
#define SR_BIT_CONTACT1  8
#define SR_BIT_BTN_CLOSE 1
#define SR_BIT_BTN_OPEN  0

/* ---- UI ---- */
#define PIN_BTN_LEARN   GPIO_NUM_3      /* SW1 / external header, active low */
#define PIN_LED_STATUS  GPIO_NUM_8      /* active high */
