/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Sleepy end device Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "zb_ntc_sleep_c6.h"

#include "esp_check.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
//#include "switch_driver.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

bool connected = false;
uint16_t custom_attr_value = 0;
adc_oneshot_unit_handle_t adc1_handle;
bool calmode = false;
esp_timer_handle_t periodic_timer;

/**
 * @note Make sure set idf.py menuconfig in zigbee component as zigbee end device!
*/
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_SLEEP_CUSTOM_ADC";

void init_ADC(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));
}

void deinit_ADC(void)
{
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}

static esp_err_t deferred_driver_init(void)
{
    start_custom_timer();
    gpio_set_direction(CALSWITCH, GPIO_MODE_INPUT);
    gpio_set_pull_mode(CALSWITCH, GPIO_FLOATING);  // added external pull-up resistor 68k

// https://github.com/espressif/esp-zigbee-sdk/issues/629  // does not compile...?
/*#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_init(CONFIG_GPIO_INPUT_IO_WAKEUP);
        rtc_gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
        rtc_gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#else
        gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
        gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#endif
*/
    return ESP_OK;
}

/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

static void update_attr_timer_callback(void *arg)
{
    static const char *TAG = "TIMER_CALLBACK";
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "Custom timer called, time since boot: %lld us  - Connected: %d", time_since_boot, connected);

    if (((gpio_get_level(CALSWITCH) == 0) & !calmode)) {  // enter calmode
        calmode = true;
        update_reporting_info();
        esp_timer_restart(periodic_timer, CAL_INTERVAL * 1000000);  // measure ADC every CAL_INTERVAL sec
        ESP_LOGI(TAG, "Entered Calibration Mode");
    }
    if (((gpio_get_level(CALSWITCH) == 1) & calmode)) {  // re-enter normal mode
        calmode = false;
        update_reporting_info();
        esp_timer_restart(periodic_timer, MEAS_INTERVAL * 1000000);
        ESP_LOGI(TAG, "Re-entered Normal Mode");
    }
    
    init_ADC();
    int adc_raw = 0;
    int adc_bit = 0;
    for (int i = 0; i < AVERAGING; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &adc_raw));
        adc_bit += adc_raw;
    }
    adc_bit /= AVERAGING;
    ESP_LOGI(TAG, "ADC Data [bit]: %d", adc_bit);
    deinit_ADC();  // without deinit and reinit ADC does only work once (caused by esp zb power safe mode?)

    custom_attr_value = (uint16_t)adc_bit;
    if (connected) {
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_status_t st = esp_zb_zcl_set_attribute_val(CUSTOM_EP,
                                   CUSTOM_CLUSTER_ID,
                                   ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                   CUSTOM_ATTR_ID, &custom_attr_value, false);
        esp_zb_lock_release();
        ESP_LOGI(TAG, "Custom attribute: %d  - Status: %d", custom_attr_value, st);
    }
}

static void start_custom_timer()
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &update_attr_timer_callback,
        .name = "custom_timer"};

//    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, MEAS_INTERVAL * 1000000));  // usec
}

void update_reporting_info() {
    uint16_t min_interval = MIN_INTERVAL;
    uint16_t max_interval = MAX_INTERVAL;
    uint16_t delta = DELTA;
    if (calmode) {
        min_interval = CAL_INTERVAL;
        max_interval = CAL_INTERVAL;
        delta = 0;
    }
    esp_zb_zcl_reporting_info_t custom_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = CUSTOM_EP,
        .cluster_id = CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .dst.endpoint = 1,
        .attr_id = CUSTOM_ATTR_ID,
        .u.send_info.def_min_interval = min_interval,
        .u.send_info.def_max_interval = max_interval,
        .u.send_info.delta.u16 = delta,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_lock_acquire(portMAX_DELAY);
    ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&custom_reporting_info));
    esp_zb_lock_release();
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                connected = true;
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            connected = true;
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
      case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        ESP_LOGI(TAG, "Zigbee can sleep");
        esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    /* Enable zigbee light sleep */
    esp_zb_sleep_enable(true);
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_sleep_set_threshold(2000);  // msec

    uint16_t custom_attr_id = CUSTOM_ATTR_ID;
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);  // 0x0000
    esp_zb_attribute_list_t *id_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);  // 0x0003
    esp_zb_attribute_list_t *cust_attr_list = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    // config basic cluster attr
    uint8_t power_source = 3;  // battery
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER));
    // config id cluster attr
    uint16_t identify_time = 0x0000;
    ESP_ERROR_CHECK(esp_zb_identify_cluster_add_attr(id_attr_list, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time));  // id 0x0000
    // config custom cluster attr // https://docs.espressif.com/projects/esp-zigbee-sdk/en/latest/esp32/user-guide/zcl_custom.html
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(cust_attr_list, custom_attr_id, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_MANUF_SPEC | ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &custom_attr_value));
    // add cluster to list
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, id_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, cust_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = CUSTOM_EP,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
        .app_device_version = 0};
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config));

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    
    update_reporting_info();

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    /* load Zigbee platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
