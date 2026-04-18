#include "bt_ascii_control.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "modbus_slave.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "BT_ASCII";

#define BT_ASCII_DEVICE_NAME "RH-AUTONOMOUS"
#define BT_ASCII_SERVER_NAME "RH_ASCII"
#define BT_ASCII_PIN_CODE "1234"
#define BT_ASCII_CMD_MAX_LEN 160
#define BT_ASCII_RESP_MAX_LEN 512
#define BT_ASCII_QUEUE_LEN 8

typedef struct {
  char text[BT_ASCII_CMD_MAX_LEN];
} bt_ascii_cmd_t;

static const esp_spp_sec_t s_sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t s_role_slave = ESP_SPP_ROLE_SLAVE;
static portMUX_TYPE s_bt_lock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_cmd_queue = NULL;
static uint32_t s_client_handle = 0;
static bool s_client_connected = false;
static bool s_initialized = false;
static char s_rx_buffer[BT_ASCII_CMD_MAX_LEN] = {0};
static size_t s_rx_len = 0;
static bool s_rx_overflow = false;

static char *bda_to_str(const uint8_t *bda, char *str, size_t size) {
  if (bda == NULL || str == NULL || size < 18U) {
    return NULL;
  }

  snprintf(str, size, "%02x:%02x:%02x:%02x:%02x:%02x", bda[0], bda[1], bda[2],
           bda[3], bda[4], bda[5]);
  return str;
}

static esp_err_t bt_ascii_send_text(const char *text) {
  if (text == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint32_t handle = 0;
  bool connected = false;
  taskENTER_CRITICAL(&s_bt_lock);
  handle = s_client_handle;
  connected = s_client_connected;
  taskEXIT_CRITICAL(&s_bt_lock);

  if (!connected || handle == 0U) {
    return ESP_ERR_INVALID_STATE;
  }

  size_t len = strlen(text);
  if (len == 0U) {
    return ESP_OK;
  }

  return esp_spp_write(handle, (int)len, (uint8_t *)text);
}

static void bt_ascii_send_response(const char *text) {
  if (text == NULL) {
    return;
  }

  char buffer[BT_ASCII_RESP_MAX_LEN + 4] = {0};
  snprintf(buffer, sizeof(buffer), "%s\r\n", text);
  esp_err_t err = bt_ascii_send_text(buffer);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "Failed to send SPP response: %s", esp_err_to_name(err));
  }
}

static void bt_ascii_reset_rx_buffer(void) {
  taskENTER_CRITICAL(&s_bt_lock);
  memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
  s_rx_len = 0U;
  s_rx_overflow = false;
  taskEXIT_CRITICAL(&s_bt_lock);
}

static void bt_ascii_queue_line(void) {
  bt_ascii_cmd_t cmd = {0};

  taskENTER_CRITICAL(&s_bt_lock);
  size_t copy_len = s_rx_len;
  if (copy_len >= sizeof(cmd.text)) {
    copy_len = sizeof(cmd.text) - 1U;
  }
  memcpy(cmd.text, s_rx_buffer, copy_len);
  cmd.text[copy_len] = '\0';
  memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
  s_rx_len = 0U;
  taskEXIT_CRITICAL(&s_bt_lock);

  if (cmd.text[0] == '\0') {
    return;
  }

  if (xQueueSend(s_cmd_queue, &cmd, 0) != pdTRUE) {
    bt_ascii_send_response("ERR command queue full");
  }
}

static void bt_ascii_process_rx_byte(uint8_t byte) {
  if (byte == '\r') {
    return;
  }

  if (byte == '\n') {
    bool overflow = false;
    size_t rx_len = 0U;

    taskENTER_CRITICAL(&s_bt_lock);
    overflow = s_rx_overflow;
    rx_len = s_rx_len;
    if (overflow) {
      memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
      s_rx_len = 0U;
      s_rx_overflow = false;
    }
    taskEXIT_CRITICAL(&s_bt_lock);

    if (overflow) {
      bt_ascii_send_response("ERR command too long");
      return;
    }

    if (rx_len > 0U) {
      bt_ascii_queue_line();
    }
    return;
  }

  if (byte < 0x20U || byte > 0x7EU) {
    return;
  }

  taskENTER_CRITICAL(&s_bt_lock);
  if (s_rx_overflow) {
    taskEXIT_CRITICAL(&s_bt_lock);
    return;
  }

  if ((s_rx_len + 1U) >= sizeof(s_rx_buffer)) {
    s_rx_overflow = true;
    taskEXIT_CRITICAL(&s_bt_lock);
    return;
  }

  s_rx_buffer[s_rx_len++] = (char)byte;
  s_rx_buffer[s_rx_len] = '\0';
  taskEXIT_CRITICAL(&s_bt_lock);
}

static void bt_ascii_worker(void *arg) {
  (void)arg;

  bt_ascii_cmd_t cmd = {0};
  char response[BT_ASCII_RESP_MAX_LEN] = {0};

  while (1) {
    if (xQueueReceive(s_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    memset(response, 0, sizeof(response));
    esp_err_t err =
        modbus_handle_ascii_command(cmd.text, response, sizeof(response));
    if (response[0] == '\0') {
      snprintf(response, sizeof(response), "ERR %s", esp_err_to_name(err));
    }
    bt_ascii_send_response(response);
  }
}

static void bt_ascii_gap_cb(esp_bt_gap_cb_event_t event,
                            esp_bt_gap_cb_param_t *param) {
  char bda_str[18] = {0};

  switch (event) {
  case ESP_BT_GAP_AUTH_CMPL_EVT:
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(TAG, "BT auth OK device=%s bda=[%s]", param->auth_cmpl.device_name,
               bda_to_str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
    } else {
      ESP_LOGW(TAG, "BT auth failed status=%d", param->auth_cmpl.stat);
    }
    break;
  case ESP_BT_GAP_MODE_CHG_EVT:
    ESP_LOGI(TAG, "BT GAP mode=%d bda=[%s]", param->mode_chg.mode,
             bda_to_str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
    break;
  default:
    break;
  }
}

static void bt_ascii_spp_cb(esp_spp_cb_event_t event,
                            esp_spp_cb_param_t *param) {
  char bda_str[18] = {0};

  switch (event) {
  case ESP_SPP_INIT_EVT:
    if (param->init.status == ESP_SPP_SUCCESS) {
      ESP_LOGI(TAG, "SPP init complete");
      esp_spp_start_srv(s_sec_mask, s_role_slave, 0, BT_ASCII_SERVER_NAME);
    } else {
      ESP_LOGE(TAG, "SPP init failed status=%d", param->init.status);
    }
    break;
  case ESP_SPP_START_EVT:
    if (param->start.status == ESP_SPP_SUCCESS) {
      ESP_LOGI(TAG, "SPP server started handle=%" PRIu32 " scn=%d",
               param->start.handle, param->start.scn);
      esp_bt_gap_set_device_name(BT_ASCII_DEVICE_NAME);
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                               ESP_BT_GENERAL_DISCOVERABLE);
    } else {
      ESP_LOGE(TAG, "SPP start failed status=%d", param->start.status);
    }
    break;
  case ESP_SPP_SRV_OPEN_EVT:
    taskENTER_CRITICAL(&s_bt_lock);
    s_client_handle = param->srv_open.handle;
    s_client_connected = (param->srv_open.status == ESP_SPP_SUCCESS);
    taskEXIT_CRITICAL(&s_bt_lock);
    bt_ascii_reset_rx_buffer();
    ESP_LOGI(TAG, "SPP client connected handle=%" PRIu32 " bda=[%s]",
             param->srv_open.handle,
             bda_to_str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
    bt_ascii_send_response("OK RH autonomous ASCII ready; type help");
    break;
  case ESP_SPP_CLOSE_EVT:
    taskENTER_CRITICAL(&s_bt_lock);
    s_client_handle = 0U;
    s_client_connected = false;
    taskEXIT_CRITICAL(&s_bt_lock);
    bt_ascii_reset_rx_buffer();
    ESP_LOGI(TAG, "SPP client disconnected handle=%" PRIu32 " status=%d",
             param->close.handle, param->close.status);
    break;
  case ESP_SPP_DATA_IND_EVT:
    for (int i = 0; i < param->data_ind.len; ++i) {
      bt_ascii_process_rx_byte(param->data_ind.data[i]);
    }
    break;
  case ESP_SPP_WRITE_EVT:
    if (param->write.status != ESP_SPP_SUCCESS) {
      ESP_LOGW(TAG, "SPP write failed handle=%" PRIu32 " status=%d",
               param->write.handle, param->write.status);
    }
    break;
  default:
    break;
  }
}

esp_err_t bt_ascii_control_init(void) {
  if (s_initialized) {
    return ESP_OK;
  }

  s_cmd_queue = xQueueCreate(BT_ASCII_QUEUE_LEN, sizeof(bt_ascii_cmd_t));
  if (s_cmd_queue == NULL) {
    return ESP_ERR_NO_MEM;
  }

  if (xTaskCreate(bt_ascii_worker, "bt_ascii", 4096, NULL, 5, NULL) !=
      pdPASS) {
    vQueueDelete(s_cmd_queue);
    s_cmd_queue = NULL;
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "BT BLE mem release returned %s", esp_err_to_name(ret));
  }

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  if (ret != ESP_OK) {
    return ret;
  }

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = esp_bluedroid_enable();
  if (ret != ESP_OK) {
    return ret;
  }

  ret = esp_bt_gap_register_callback(bt_ascii_gap_cb);
  if (ret != ESP_OK) {
    return ret;
  }

  ret = esp_spp_register_callback(bt_ascii_spp_cb);
  if (ret != ESP_OK) {
    return ret;
  }

  esp_bt_pin_code_t pin_code = {BT_ASCII_PIN_CODE[0], BT_ASCII_PIN_CODE[1],
                                BT_ASCII_PIN_CODE[2], BT_ASCII_PIN_CODE[3]};
  ret = esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
  if (ret != ESP_OK) {
    return ret;
  }

  esp_spp_cfg_t spp_cfg = {
      .mode = ESP_SPP_MODE_CB,
      .enable_l2cap_ertm = true,
      .tx_buffer_size = 0,
  };
  ret = esp_spp_enhanced_init(&spp_cfg);
  if (ret != ESP_OK) {
    return ret;
  }

  s_initialized = true;
  ESP_LOGI(TAG, "Bluetooth ASCII control ready: device=%s pin=%s",
           BT_ASCII_DEVICE_NAME, BT_ASCII_PIN_CODE);
  return ESP_OK;
}
