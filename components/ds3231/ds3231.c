#include "ds3231.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <stdlib.h>

#define DS3231_REG_SECONDS 0x00
#define DS3231_REG_CONTROL 0x0E
#define DS3231_EOSC_BIT 0x80

typedef struct ds3231_dev_t {
  i2c_port_t i2c_port;
  uint8_t i2c_addr;
} ds3231_dev_t;

static const char *TAG = "DS3231";

static uint8_t bcd_to_bin(uint8_t bcd) {
  return (uint8_t)(((bcd >> 4U) * 10U) + (bcd & 0x0FU));
}

static uint8_t bin_to_bcd(uint8_t bin) {
  return (uint8_t)(((bin / 10U) << 4U) | (bin % 10U));
}

static esp_err_t ds3231_write_reg(ds3231_handle_t handle, uint8_t reg,
                                  const uint8_t *data, size_t len) {
  if (handle == NULL || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t *buf = (uint8_t *)malloc(len + 1U);
  if (buf == NULL) {
    return ESP_ERR_NO_MEM;
  }

  buf[0] = reg;
  for (size_t i = 0; i < len; ++i) {
    buf[i + 1U] = data[i];
  }

  esp_err_t err = i2c_master_write_to_device(handle->i2c_port, handle->i2c_addr,
                                             buf, len + 1U, pdMS_TO_TICKS(100));
  free(buf);
  return err;
}

static esp_err_t ds3231_read_reg(ds3231_handle_t handle, uint8_t reg,
                                 uint8_t *data, size_t len) {
  if (handle == NULL || data == NULL || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  return i2c_master_write_read_device(handle->i2c_port, handle->i2c_addr, &reg,
                                      1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t ds3231_init(const ds3231_config_t *cfg, ds3231_handle_t *out_handle) {
  if (cfg == NULL || out_handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  ds3231_dev_t *dev = (ds3231_dev_t *)calloc(1, sizeof(ds3231_dev_t));
  if (dev == NULL) {
    return ESP_ERR_NO_MEM;
  }

  dev->i2c_port = cfg->i2c_port;
  dev->i2c_addr = (cfg->i2c_addr == 0) ? DS3231_I2C_ADDR : cfg->i2c_addr;

  uint8_t control = 0;
  esp_err_t err = ds3231_read_reg(dev, DS3231_REG_CONTROL, &control, 1);
  if (err != ESP_OK) {
    free(dev);
    return err;
  }

  control &= (uint8_t)~DS3231_EOSC_BIT;
  err = ds3231_write_reg(dev, DS3231_REG_CONTROL, &control, 1);
  if (err != ESP_OK) {
    free(dev);
    return err;
  }

  *out_handle = dev;
  ESP_LOGI(TAG, "DS3231 initialized on I2C addr 0x%02X", dev->i2c_addr);
  return ESP_OK;
}

esp_err_t ds3231_get_time(ds3231_handle_t handle, ds3231_time_t *time) {
  if (handle == NULL || time == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t raw[3] = {0};
  esp_err_t err = ds3231_read_reg(handle, DS3231_REG_SECONDS, raw, sizeof(raw));
  if (err != ESP_OK) {
    return err;
  }

  uint8_t sec = bcd_to_bin((uint8_t)(raw[0] & 0x7FU));
  uint8_t min = bcd_to_bin((uint8_t)(raw[1] & 0x7FU));
  uint8_t hour = bcd_to_bin((uint8_t)(raw[2] & 0x3FU));

  if (sec >= 60U || min >= 60U || hour >= 24U) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  time->second = sec;
  time->minute = min;
  time->hour = hour;
  return ESP_OK;
}

esp_err_t ds3231_set_time(ds3231_handle_t handle, const ds3231_time_t *time) {
  if (handle == NULL || time == NULL) {
    return ESP_ERR_INVALID_ARG;
  }
  if (time->second >= 60U || time->minute >= 60U || time->hour >= 24U) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t raw[3] = {
      bin_to_bcd(time->second),
      bin_to_bcd(time->minute),
      bin_to_bcd(time->hour),
  };

  return ds3231_write_reg(handle, DS3231_REG_SECONDS, raw, sizeof(raw));
}
