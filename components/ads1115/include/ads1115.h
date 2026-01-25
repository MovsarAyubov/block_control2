#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Common I2C Addresses
#define ADS1115_ADDR_GND 0x48
#define ADS1115_ADDR_VDD 0x49
#define ADS1115_ADDR_SDA 0x4A
#define ADS1115_ADDR_SCL 0x4B

typedef struct ads1115_config_t {
  i2c_port_t i2c_port;
  uint8_t i2c_addr;
} ads1115_config_t;

typedef struct ads1115_context_t *ads1115_handle_t;

/**
 * @brief Initialize ADS1115 driver
 *
 * @param config Pointer to configuration struct
 * @param ret_handle Pointer to return handle
 * @return esp_err_t
 */
esp_err_t ads1115_init(const ads1115_config_t *config,
                       ads1115_handle_t *ret_handle);

/**
 * @brief Read voltage from a specific channel (Single-ended)
 *
 * @param handle Driver handle
 * @param channel ADS1115 Channel (0-3)
 * @param out_voltage_mv Pointer to store result in mV
 * @return esp_err_t
 */
esp_err_t ads1115_read_voltage(ads1115_handle_t handle, int channel,
                               float *out_voltage_mv);

/**
 * @brief Free resources
 *
 * @param handle Driver handle
 * @return esp_err_t
 */
esp_err_t ads1115_del(ads1115_handle_t handle);

#ifdef __cplusplus
}
#endif
