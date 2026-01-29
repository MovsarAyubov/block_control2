#pragma once

#include "ads1115.h"
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // I2C / ADS1115 definition
  i2c_port_t i2c_port;
  uint8_t ads_addr;
  // ads_channel removed as we use hardcoded differential pair A2-A3
  float shunt_resistor_ohm; // Resistor value (e.g., 109.3)

  // Motor Control GPIOs
  gpio_num_t pin_open;  // Relay for OPEN
  gpio_num_t pin_close; // Relay for CLOSE

  // Control Parameters
  float hysteresis_percent; // Deadband (e.g., 1.0%)
} rll400_config_t;

typedef struct rll400_context_t *rll400_handle_t;

/**
 * @brief Initialize RLL400 controller
 *
 * @param config Configuration struct
 * @param ret_handle Returned handle
 * @return esp_err_t
 */
esp_err_t rll400_init(const rll400_config_t *config,
                      rll400_handle_t *ret_handle);

/**
 * @brief Set target position in percent (0-100)
 *
 * @param handle Handle
 * @param target_percent Target position
 * @return esp_err_t
 */
esp_err_t rll400_set_target(rll400_handle_t handle, float target_percent);

/**
 * @brief Get current status
 *
 * @param handle Handle
 * @param out_position_percent Current position (calculated)
 * @param out_current_ma Current loop current (mA)
 * @return esp_err_t
 */
esp_err_t rll400_get_status(rll400_handle_t handle, float *out_position_percent,
                            float *out_current_ma);

/**
 * @brief Run the control loop. Should be called periodically or run in a
 * background task within the component. For simplicity, this function performs
 * one iteration of reading -> control.
 *
 * @param handle Handle
 * @return esp_err_t
 */
esp_err_t rll400_process(rll400_handle_t handle);

/**
 * @brief Stop motor immediately
 *
 * @param handle Handle
 * @return esp_err_t
 */
esp_err_t rll400_stop(rll400_handle_t handle);

/**
 * @brief Free resources
 *
 * @param handle Handle
 * @return esp_err_t
 */
esp_err_t rll400_del(rll400_handle_t handle);

#ifdef __cplusplus
}
#endif
