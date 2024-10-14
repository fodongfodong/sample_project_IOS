#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "nvs_flash.h"

#define TXD_PIN (17)
#define RXD_PIN (16)
#define UART_NUM UART_NUM_2
#define GPS_RESET_PIN (5)
#define NMEA_MAX_LENGTH (128)
#define SERVICE_UUID "12345678-1234-5678-1234-56789abcdef0"
#define CHAR_UUID "12345678-1234-5678-1234-56789abcdef1"

static const char *TAG = "BLE_GPS";
static uint16_t ble_service_handle;
static uint16_t ble_char_handle;


esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, NMEA_MAX_LENGTH * 2, 0, 0, NULL, 0);
}

void gps_task(void *arg) {
    uint8_t data[NMEA_MAX_LENGTH];
    while (1) {
        int length = uart_read_bytes(UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        if (length > 0) {
            data[length] = '\0';
            if (strncmp((char *)data, "$GNGGA", 6) == 0) {
                ESP_LOGI(TAG, "NMEA message: %s", (char *)data);
                esp_ble_gatts_set_attr_value(ble_char_handle, length, data);
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    if (event == ESP_GAP_BLE_ADV_START_COMPLETE_EVT) {
        ESP_LOGI(TAG, "BLE advertising started.");
    }
}

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        esp_ble_gap_start_advertising(&adv_params);
    } else if (event == ESP_GATTS_ADD_CHAR_EVT) {
        ble_char_handle = param->add_char.attr_handle;
    }
}

void ble_init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_ble_gatts_register_callback(gatts_profile_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);
}

void app_main() {
    init_uart();
    ble_init();

    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}
