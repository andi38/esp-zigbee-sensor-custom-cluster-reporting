#include "esp_zigbee_core.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false    /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   10000    /* 4000 millisecond */
//#define HA_ESP_LIGHT_ENDPOINT           10    /* esp light bulb device endpoint, used to process light controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/* Custom cluster config */
#define CUSTOM_EP 90
#define CUSTOM_CLUSTER_ID 0xff00
#define CUSTOM_ATTR_ID 1
#define MIN_INTERVAL 1*60
#define MAX_INTERVAL 15*60
#define DELTA 20

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""espressif"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07""esp32c6" /* Customized model identifier */

/* ADC measurement config */
#define MEAS_INTERVAL 60
#define AVERAGING 5

/* Calibration mode switch */
#define CALSWITCH GPIO_NUM_3
#define CAL_INTERVAL 10

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

static void update_attr_timer_callback(void* arg);
static void start_custom_timer();
void init_ADC();
void deinit_ADC(void);
void update_reporting_info(void);
