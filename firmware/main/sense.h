/* ADC access: motor current sense and supply-rail measurement. */
#pragma once

#include <stdint.h>
#include "hal/adc_types.h"

void sense_init(void);

/* Raw pin voltage in mV (calibrated when the cal scheme is available). */
int sense_read_mv(adc_channel_t ch);

/* Supply rail in mV (divider corrected). */
int sense_read_vin_mv(void);

/* Motor current in mA derived from a CS pin voltage in mV (typ. K). */
int sense_cs_mv_to_ma(int cs_mv);
