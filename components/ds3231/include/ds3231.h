#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DS3231_I2C_ADDR 0x68

typedef struct {
  i2c_port_t i2c_port;
  uint8_t i2c_addr;
} ds3231_config_t;

typedef struct ds3231_dev_t *ds3231_handle_t;

typedef struct {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} ds3231_time_t;

esp_err_t ds3231_init(const ds3231_config_t *cfg, ds3231_handle_t *out_handle);
esp_err_t ds3231_get_time(ds3231_handle_t handle, ds3231_time_t *time);
esp_err_t ds3231_set_time(ds3231_handle_t handle, const ds3231_time_t *time);

#ifdef __cplusplus
}
#endif
