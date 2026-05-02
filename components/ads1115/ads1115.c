#include "ads1115.h"
#include "esp_check.h"
#include "esp_log.h"
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
  i2c_master_dev_handle_t dev_handle;
} ads1115_context_t;

static esp_err_t _write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg,
                            uint16_t value) {
  uint8_t data[3] = {reg, (uint8_t)((value >> 8) & 0xFF),
                     (uint8_t)(value & 0xFF)};
  esp_err_t ret = i2c_master_transmit(dev_handle, data, sizeof(data), 500);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C Write Failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

static esp_err_t _read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg,
                           int16_t *val) {
  uint8_t data[2] = {0};
  esp_err_t ret =
      i2c_master_transmit_receive(dev_handle, &reg, 1, data, sizeof(data), 500);

  if (ret == ESP_OK) {
    *val = (int16_t)((data[0] << 8) | data[1]);
  } else {
    ESP_LOGE(TAG, "I2C Read Failed: %s", esp_err_to_name(ret));
  }
  return ret;
}

esp_err_t ads1115_init(const ads1115_config_t *config,
                       ads1115_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid args");
  ESP_RETURN_ON_FALSE(config->i2c_bus, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid I2C bus");

  ads1115_context_t *ctx = calloc(1, sizeof(ads1115_context_t));
  if (!ctx)
    return ESP_ERR_NO_MEM;

  memcpy(&ctx->config, config, sizeof(ads1115_config_t));

  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = config->i2c_addr,
      .scl_speed_hz = 100000,
  };
  esp_err_t err = i2c_master_bus_add_device(config->i2c_bus, &dev_cfg,
                                            &ctx->dev_handle);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

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

  ESP_RETURN_ON_ERROR(_write_reg(ctx->dev_handle, ADS_REG_CFG, config),
                      TAG, "Write config failed");

  vTaskDelay(pdMS_TO_TICKS(20)); // Conversion time

  int16_t raw_val = 0;
  ESP_RETURN_ON_ERROR(_read_reg(ctx->dev_handle, ADS_REG_CONV, &raw_val),
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

  ESP_RETURN_ON_ERROR(_write_reg(ctx->dev_handle, ADS_REG_CFG, config),
                      TAG, "Write config failed");

  vTaskDelay(pdMS_TO_TICKS(20)); // Conversion time

  int16_t raw_val = 0;
  ESP_RETURN_ON_ERROR(_read_reg(ctx->dev_handle, ADS_REG_CONV, &raw_val),
                      TAG, "Read val failed");

  *out_voltage_mv = raw_val * 0.125f;
  return ESP_OK;
}

esp_err_t ads1115_del(ads1115_handle_t handle) {
  if (handle) {
    ads1115_context_t *ctx = handle;
    if (ctx->dev_handle) {
      (void)i2c_master_bus_rm_device(ctx->dev_handle);
    }
    free(ctx);
  }
  return ESP_OK;
}
