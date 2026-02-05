#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for 3-way Valve
 */
typedef struct {
  gpio_num_t gpio_open_num;   /*!< GPIO pin to open the valve */
  gpio_num_t gpio_close_num;  /*!< GPIO pin to close the valve */
  float temp_open_threshold;  /*!< Temperature threshold to open valve (<=) */
  float temp_close_threshold; /*!< Temperature threshold to close valve (>=) */
} valve_3way_config_t;

/**
 * @brief Opaque handle for valve instance
 */
typedef struct valve_3way_ctx_t *valve_3way_handle_t;

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
 * @brief Process Valve Logic based on current temperature
 *
 * This function checks the temperature against thresholds and controls
 * the GPIOs accordingly, ensuring interlock protection.
 *
 * @param[in] handle Valve handle
 * @param[in] current_temp_c Current temperature in Celsius
 * @return ESP_OK on success
 */
esp_err_t valve_3way_process(valve_3way_handle_t handle, float current_temp_c);

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
