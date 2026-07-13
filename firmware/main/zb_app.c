#include "zb_app.h"

#include <string.h>

#include "shutter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

static const char *TAG = "zb";

#define EP_COVER           1
#define EP_CONTACT_FIRST   2   /* endpoints 2..5: window contacts 1..4 */
#define EP_TAMPER_FIRST    6   /* endpoints 6..9: tamper loops 1..4 */

/* ZCL character strings are length-prefixed */
#define MANUFACTURER_NAME  "\x05""MrDix"
#define MODEL_IDENTIFIER   "\x0b""ShutterNode"

static bool s_joined;

/* ---------------- commissioning ---------------- */

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "factory new, starting network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                s_joined = true;
                ESP_LOGI(TAG, "rejoined network (short: 0x%04hx)",
                         esp_zb_get_short_address());
            }
        } else {
            ESP_LOGW(TAG, "stack %s failed (%s), retrying",
                     esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            s_joined = true;
            esp_zb_ieee_addr_t addr;
            esp_zb_get_long_address(addr);
            ESP_LOGI(TAG,
                     "joined (ieee %02x%02x%02x%02x%02x%02x%02x%02x, "
                     "pan 0x%04hx, ch %d, short 0x%04hx)",
                     addr[7], addr[6], addr[5], addr[4],
                     addr[3], addr[2], addr[1], addr[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(),
                     esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "steering failed (%s), retrying in 1 s",
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm(bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        s_joined = false;
        ESP_LOGW(TAG, "left the network, starting steering");
        esp_zb_scheduler_alarm(bdb_start_top_level_commissioning_cb,
                               ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        break;

    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

/* ---------------- incoming commands ---------------- */

static esp_err_t wc_movement_handler(const esp_zb_zcl_window_covering_movement_message_t *msg)
{
    ESP_RETURN_ON_FALSE(msg, ESP_FAIL, TAG, "empty movement message");
    ESP_RETURN_ON_FALSE(msg->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG,
                        TAG, "movement message status error (%d)", msg->info.status);

    switch (msg->command) {
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:
        ESP_LOGI(TAG, "cmd: open");
        shutter_open();
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE:
        ESP_LOGI(TAG, "cmd: close");
        shutter_close();
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:
        ESP_LOGI(TAG, "cmd: stop");
        shutter_stop();
        break;
    case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
        ESP_LOGI(TAG, "cmd: go to %d %%", msg->payload.percentage_lift_value);
        shutter_goto_percent(msg->payload.percentage_lift_value);
        break;
    default:
        ESP_LOGW(TAG, "unhandled covering command 0x%x", msg->command);
        break;
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID:
        return wc_movement_handler(
            (const esp_zb_zcl_window_covering_movement_message_t *)message);
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return ESP_OK;
    default:
        ESP_LOGD(TAG, "unhandled action callback 0x%x", callback_id);
        return ESP_OK;
    }
}

/* ---------------- endpoint construction ---------------- */

static esp_zb_cluster_list_t *create_cover_clusters(void)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_basic_cluster_cfg_t basic_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x01, /* mains */
    };
    esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(&basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
        basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
        cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(
        cl, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_window_covering_cluster_cfg_t wc_cfg = {
        .covering_type = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_TYPE_SHUTTER,
        .covering_status = ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_OPERATIONAL |
                           ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CONFIG_LIFT_CONTROL_IS_CLOSED_LOOP,
        .covering_mode = 0,
    };
    esp_zb_attribute_list_t *wc = esp_zb_window_covering_cluster_create(&wc_cfg);
    uint8_t lift_pct = 100; /* boot assumption: fully closed */
    ESP_ERROR_CHECK(esp_zb_window_covering_cluster_add_attr(
        wc, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
        &lift_pct));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_window_covering_cluster(
        cl, wc, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    /* supply-rail voltage as an analog input */
    esp_zb_analog_input_cluster_cfg_t ai_cfg = {
        .out_of_service = false,
        .status_flags = 0,
        .present_value = 0.0f,
    };
    esp_zb_attribute_list_t *ai = esp_zb_analog_input_cluster_create(&ai_cfg);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(
        cl, ai, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cl;
}

static esp_zb_cluster_list_t *create_binary_input_clusters(const char *description,
                                                           bool initial)
{
    esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

    esp_zb_binary_input_cluster_cfg_t bi_cfg = {
        .out_of_service = false,
        .status_flags = 0,
    };
    esp_zb_attribute_list_t *bi = esp_zb_binary_input_cluster_create(&bi_cfg);
    bool present = initial;
    ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(
        bi, ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &present));
    ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(
        bi, ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID, (void *)description));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(
        cl, bi, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cl;
}

static void register_device(void)
{
    esp_zb_ep_list_t *eps = esp_zb_ep_list_create();

    esp_zb_endpoint_config_t cover_ep = {
        .endpoint = EP_COVER,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(eps, create_cover_clusters(), cover_ep));

    /* ZCL length-prefixed description strings */
    static const char *contact_desc[4] = {
        "\x08""contact1", "\x08""contact2", "\x08""contact3", "\x08""contact4"
    };
    static const char *tamper_desc[4] = {
        "\x07""tamper1", "\x07""tamper2", "\x07""tamper3", "\x07""tamper4"
    };

    for (int i = 0; i < 4; i++) {
        esp_zb_endpoint_config_t ep = {
            .endpoint = EP_CONTACT_FIRST + i,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(
            eps, create_binary_input_clusters(contact_desc[i], false), ep));
    }
    for (int i = 0; i < 4; i++) {
        esp_zb_endpoint_config_t ep = {
            .endpoint = EP_TAMPER_FIRST + i,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(
            eps, create_binary_input_clusters(tamper_desc[i], false), ep));
    }

    ESP_ERROR_CHECK(esp_zb_device_register(eps));
}

/* ---------------- zigbee main task ---------------- */

static void zb_task(void *arg)
{
    esp_zb_cfg_t cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,
        .install_code_policy = false,
        .nwk_cfg.zczr_cfg = {
            .max_children = 10,
        },
    };
    esp_zb_init(&cfg);
    register_device();
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void zb_app_start(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = { .radio_mode = ZB_RADIO_MODE_NATIVE },
        .host_config = { .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(zb_task, "zigbee", 8192, NULL, 5, NULL);
}

bool zb_app_joined(void) { return s_joined; }

void zb_app_factory_reset(void)
{
    ESP_LOGW(TAG, "factory reset requested");
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_factory_reset();
    esp_zb_lock_release();
}

/* ---------------- attribute updates ---------------- */

static void set_attr(uint8_t ep, uint16_t cluster, uint16_t attr, void *value)
{
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(ep, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 attr, value, false);
    esp_zb_lock_release();
}

void zb_app_update_position(uint8_t pct_closed)
{
    set_attr(EP_COVER, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
             ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID,
             &pct_closed);
}

void zb_app_update_contact(uint8_t index, bool closed, bool tamper)
{
    if (index >= 4) {
        return;
    }
    bool contact_val = closed;
    bool tamper_val = tamper;
    set_attr(EP_CONTACT_FIRST + index, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
             ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &contact_val);
    set_attr(EP_TAMPER_FIRST + index, ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
             ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &tamper_val);
}

void zb_app_update_vin(int mv)
{
    if (mv < 0) {
        return;
    }
    float volts = mv / 1000.0f;
    set_attr(EP_COVER, ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
             ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &volts);
}
