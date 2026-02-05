#include "ads1115.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ads1115";

// Register addresses
#define ADS_REG_CONV 0x00
#define ADS_REG_CFG 0x01

// Configuration bits
#define ADS_CFG_OS_SINGLE (0x8000)
#define ADS_CFG_MUX_OFFSET (12)
#define ADS_CFG_PGA_4_096V (0x0200) // Gain=1 (+/-4.096V)
#define ADS_CFG_MODE_SINGLE (0x0100)
#define ADS_CFG_DR_128SPS (0x0080)
#define ADS_CFG_COMP_QUE_DISABLE (0x0003)

typedef struct ads1115_context_t {
  ads1115_config_t config;
} ads1115_context_t;

static esp_err_t _write_reg(i2c_port_t port, uint8_t addr, uint8_t reg,
                            uint16_t value) {
  // ESP_LOGD(TAG, "Write Reg: Addr 0x%02X, Reg 0x%02X, Val 0x%04X", addr, reg,
  // value);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true);
  i2c_master_write_byte(cmd, value & 0xFF, true);
  i2c_master_stop(cmd);
  // Increased timeout to 500ms
  esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(500));
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C Write Failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

static esp_err_t _read_reg(i2c_port_t port, uint8_t addr, uint8_t reg,
                           int16_t *val) {
  // ESP_LOGD(TAG, "Read Reg: Addr 0x%02X, Reg 0x%02X", addr, reg);
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  // Restart logic
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  uint8_t msb, lsb;
  i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  // Increased timeout to 500ms
  esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(500));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK) {
    *val = (int16_t)((msb << 8) | lsb);
  } else {
    ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t ads1115_init(const ads1115_config_t *config,
                       ads1115_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid args");

  ads1115_context_t *ctx = calloc(1, sizeof(ads1115_context_t));
  if (!ctx)
    return ESP_ERR_NO_MEM;

  memcpy(&ctx->config, config, sizeof(ads1115_config_t));
  *ret_handle = ctx;
  return ESP_OK;
}

esp_err_t ads1115_read_voltage(ads1115_handle_t handle, int channel,
                               float *out_voltage_mv) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  ads1115_context_t *ctx = handle;

  // Configure MUX for single-ended channel
  // 0 -> 100 (AIN0), 1 -> 101 (AIN1), 2 -> 110 (AIN2), 3 -> 111 (AIN3)
  uint16_t mux = (0x4 + channel) << ADS_CFG_MUX_OFFSET;

  uint16_t config = ADS_CFG_OS_SINGLE | mux | ADS_CFG_PGA_4_096V |
                    ADS_CFG_MODE_SINGLE | ADS_CFG_DR_128SPS |
                    ADS_CFG_COMP_QUE_DISABLE;

  ESP_RETURN_ON_ERROR(_write_reg(ctx->config.i2c_port, ctx->config.i2c_addr,
                                 ADS_REG_CFG, config),
                      TAG, "Write config failed");

  vTaskDelay(pdMS_TO_TICKS(20)); // Conversion time

  int16_t raw_val = 0;
  ESP_RETURN_ON_ERROR(_read_reg(ctx->config.i2c_port, ctx->config.i2c_addr,
                                ADS_REG_CONV, &raw_val),
                      TAG, "Read val failed");

  // Convert to mV (PGA=1 => 4.096V range => 0.125mV per bit)
  *out_voltage_mv = raw_val * 0.125f;

  return ESP_OK;
}

esp_err_t ads1115_read_voltage_differential(ads1115_handle_t handle,
                                            int channel_pos, int channel_neg,
                                            float *out_voltage_mv) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  ads1115_context_t *ctx = handle;

  // MUX Table for Diff:
  // 000 : AINP = AIN0 and AINN = AIN1  (Default)
  // 001 : AINP = AIN0 and AINN = AIN3
  // 010 : AINP = AIN1 and AINN = AIN3
  // 011 : AINP = AIN2 and AINN = AIN3

  uint16_t mux = 0;
  if (channel_pos == 0 && channel_neg == 1) {
    mux = 0x0 << ADS_CFG_MUX_OFFSET;
  } else if (channel_pos == 0 && channel_neg == 3) {
    mux = 0x1 << ADS_CFG_MUX_OFFSET;
  } else if (channel_pos == 1 && channel_neg == 3) {
    mux = 0x2 << ADS_CFG_MUX_OFFSET;
  } else if (channel_pos == 2 && channel_neg == 3) {
    mux = 0x3 << ADS_CFG_MUX_OFFSET;
  } else {
    // Fallback or error? defaulting to 0-1
    return ESP_ERR_INVALID_ARG;
  }

  uint16_t config = ADS_CFG_OS_SINGLE | mux | ADS_CFG_PGA_4_096V |
                    ADS_CFG_MODE_SINGLE | ADS_CFG_DR_128SPS |
                    ADS_CFG_COMP_QUE_DISABLE;

  ESP_RETURN_ON_ERROR(_write_reg(ctx->config.i2c_port, ctx->config.i2c_addr,
                                 ADS_REG_CFG, config),
                      TAG, "Write config failed");

  vTaskDelay(pdMS_TO_TICKS(20)); // Conversion time

  int16_t raw_val = 0;
  ESP_RETURN_ON_ERROR(_read_reg(ctx->config.i2c_port, ctx->config.i2c_addr,
                                ADS_REG_CONV, &raw_val),
                      TAG, "Read val failed");

  *out_voltage_mv = raw_val * 0.125f;
  return ESP_OK;
}

esp_err_t ads1115_del(ads1115_handle_t handle) {
  if (handle)
    free(handle);
  return ESP_OK;
}
