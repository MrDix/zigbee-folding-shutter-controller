idf_component_register(
    SRCS
        "main.c"
        "shutter.c"
        "inputs.c"
        "sense.c"
        "zb_app.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_adc nvs_flash esp_timer
)
