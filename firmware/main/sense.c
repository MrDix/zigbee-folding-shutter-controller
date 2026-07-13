#include "sense.h"

#include "board.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "sense";

static adc_oneshot_unit_handle_t s_unit;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;

void sense_init(void)
{
    adc_oneshot_unit_init_cfg_t ucfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &s_unit));

    adc_oneshot_chan_cfg_t ccfg = {
        .atten = ADC_ATTEN_DB_12,       /* full 0..~3.3 V range */
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_unit, ADC_CH_M1_CS, &ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_unit, ADC_CH_M2_CS, &ccfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_unit, ADC_CH_VIN, &ccfg));

    adc_cali_curve_fitting_config_t cal = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_cali_ok = (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) == ESP_OK);
    if (!s_cali_ok) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw scaling");
    }
}

int sense_read_mv(adc_channel_t ch)
{
    int raw = 0;
    if (adc_oneshot_read(s_unit, ch, &raw) != ESP_OK) {
        return -1;
    }
    if (s_cali_ok) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            return mv;
        }
    }
    /* Fallback: 12-bit full scale ~= 3300 mV at 12 dB attenuation */
    return (raw * 3300) / 4095;
}

int sense_read_vin_mv(void)
{
    int mv = sense_read_mv(ADC_CH_VIN);
    return (mv < 0) ? -1 : mv * VIN_DIV_NUM;
}

int sense_cs_mv_to_ma(int cs_mv)
{
    if (cs_mv < 0) {
        return -1;
    }
    /* I_out = V_cs / R_sense * K  ->  mA = mV * K / R */
    return (int)((int64_t)cs_mv * CS_K_TYP / CS_RSENSE_OHM);
}
