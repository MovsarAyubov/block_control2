#pragma once

#include "driver/gpio.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for 3-way Valve
 */
typedef struct {
  const char *name;            /*!< Optional valve name for logs */
  gpio_num_t gpio_open_num;    /*!< GPIO pin to open the valve */
  gpio_num_t gpio_close_num;   /*!< GPIO pin to close the valve */
  float hysteresis_c;          /*!< +/- hysteresis around setpoint */
  float initial_setpoint_c;    /*!< Initial setpoint */
  float initial_actual_temp_c; /*!< Initial measured temperature */
} valve_3way_config_t;

/**
 * @brief Opaque handle for one physical 3-way valve instance
 *
 * One handle controls exactly one valve. If the device has two valves,
 * create two independent handles and update each one with its own setpoint
 * and measured temperature.
 */
typedef struct valve_3way_ctx_t *valve_3way_handle_t;

typedef enum {
  VALVE_3WAY_STATE_STOPPED = 0,
  VALVE_3WAY_STATE_OPENING,
  VALVE_3WAY_STATE_CLOSING,
} valve_3way_state_t;

/**
 * @brief Initialize the 3-Way Valve Controller
 *
 * @param[in] config Pointer to configuration struct
 * @param[out] ret_handle Returned valve handle
 * @return ESP_OK on success
 */
esp_err_t valve_3way_init(const valve_3way_config_t *config,
                          valve_3way_handle_t *ret_handle);

/**
 * @brief Update current setpoint temperature
 *
 * @param[in] handle Valve handle
 * @param[in] setpoint_c Setpoint in Celsius
 * @return ESP_OK on success
 */
esp_err_t valve_3way_set_setpoint_c(valve_3way_handle_t handle,
                                    float setpoint_c);

/**
 * @brief Update current measured temperature
 *
 * @param[in] handle Valve handle
 * @param[in] actual_temp_c Measured temperature in Celsius
 * @return ESP_OK on success
 */
esp_err_t valve_3way_set_actual_temp_c(valve_3way_handle_t handle,
                                       float actual_temp_c);

/**
 * @brief Update hysteresis in Celsius
 *
 * @param[in] handle Valve handle
 * @param[in] hysteresis_c Hysteresis around setpoint
 * @return ESP_OK on success
 */
esp_err_t valve_3way_set_hysteresis_c(valve_3way_handle_t handle,
                                      float hysteresis_c);

/**
 * @brief Apply automatic valve logic using internally stored temperatures
 *
 * @param[in] handle Valve handle
 * @return ESP_OK on success
 */
esp_err_t valve_3way_process(valve_3way_handle_t handle);

/**
 * @brief Convenience helper: update temperatures and apply automatic logic
 *
 * Logic:
 * - if actual_temp > setpoint + hysteresis -> CLOSE
 * - if actual_temp < setpoint - hysteresis -> OPEN
 * - otherwise -> STOP
 *
 * @param[in] handle Valve handle
 * @param[in] setpoint_c Setpoint from Modbus/server in Celsius
 * @param[in] actual_temp_c Measured pipe temperature in Celsius
 * @return ESP_OK on success
 */
esp_err_t valve_3way_process_temperatures(valve_3way_handle_t handle,
                                          float setpoint_c,
                                          float actual_temp_c);

/**
 * @brief Force valve OPEN command
 *
 * Interlock protection always disables CLOSE output before enabling OPEN.
 */
esp_err_t valve_3way_open(valve_3way_handle_t handle);

/**
 * @brief Force valve CLOSE command
 *
 * Interlock protection always disables OPEN output before enabling CLOSE.
 */
esp_err_t valve_3way_close(valve_3way_handle_t handle);

/**
 * @brief Stop valve movement and disable both outputs
 */
esp_err_t valve_3way_stop(valve_3way_handle_t handle);

/**
 * @brief Get current valve state
 */
valve_3way_state_t valve_3way_get_state(valve_3way_handle_t handle);

/**
 * @brief Get current setpoint
 */
float valve_3way_get_setpoint_c(valve_3way_handle_t handle);

/**
 * @brief Get current measured temperature
 */
float valve_3way_get_actual_temp_c(valve_3way_handle_t handle);

/**
 * @brief Deinitialize and free resources
 *
 * @param[in] handle Valve handle
 * @return ESP_OK on success
 */
esp_err_t valve_3way_del(valve_3way_handle_t handle);

#ifdef __cplusplus
}
#endif
