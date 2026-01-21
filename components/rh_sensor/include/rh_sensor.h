#pragma once

#include "driver/i2c.h"
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

// ADS1115 Address (ADDR -> GND)
#define ADS1115_ADDR_GND 0x48

/**
 * @brief Configuration for RH Sensor (ADS1115)
 */
typedef struct {
  i2c_port_t i2c_port; /*!< I2C Port Number */
  uint8_t i2c_addr;    /*!< ADS1115 Address (e.g. 0x48) */
  uint8_t ads_channel; /*!< ADS1115 Channel (e.g., 0 for AIN0) */
  float r1_ohm;        /*!< Upper divider resistor */
  float r2_ohm;        /*!< Lower divider resistor */
  int offset_mv;       /*!< Calibration offset in mV */
} rh_sensor_config_t;

/**
 * @brief Opaque handle for sensor instance
 */
typedef struct rh_sensor_ctx_t *rh_sensor_handle_t;

/**
 * @brief Initialize the RH Sensor
 *
 * @param[in] config Pointer to configuration struct
 * @param[out] ret_handle Returned sensor handle
 * @return ESP_OK on success
 */
esp_err_t rh_sensor_init(const rh_sensor_config_t *config,
                         rh_sensor_handle_t *ret_handle);

/**
 * @brief Read data from the sensor
 *
 * @param[in] handle Sensor handle
 * @param[out] out_pin_mv Voltage at the ADS1115 Pin (mV)
 * @param[out] out_sensor_v Calculated sensor output voltage (V) before divider
 * @param[out] out_rh Relative Humidity (%)
 * @return ESP_OK on success
 */
esp_err_t rh_sensor_read(rh_sensor_handle_t handle, float *out_pin_mv,
                         float *out_sensor_v, float *out_rh);

/**
 * @brief Deinitialize and free resources
 *
 * @param[in] handle Sensor handle
 * @return ESP_OK on success
 */
esp_err_t rh_sensor_del(rh_sensor_handle_t handle);

#ifdef __cplusplus
}
#endif
