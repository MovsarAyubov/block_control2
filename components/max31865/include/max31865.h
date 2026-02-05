#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration for MAX31865
 */
typedef struct {
    spi_host_device_t host;   /*!< SPI Host (SPI2_HOST or SPI3_HOST) */
    int mosi_io_num;          /*!< MOSI GPIO pin */
    int miso_io_num;          /*!< MISO GPIO pin */
    int sclk_io_num;          /*!< SCLK GPIO pin */
    int cs_io_num;            /*!< CS GPIO pin */
    float r_ref;              /*!< Reference Resistor Value (Ohms) */
    float r0;                 /*!< RTD Nominal Resistance at 0°C (Ohms) (e.g. 100 for PT100, 500 for PT500) */
    bool three_wire;          /*!< Set to true for 3-wire connection */
} max31865_config_t;

/**
 * @brief Opaque handle for sensor instance
 */
typedef struct max31865_ctx_t *max31865_handle_t;

/**
 * @brief Initialize the MAX31865 Sensor
 *
 * @param[in] config Pointer to configuration struct
 * @param[out] ret_handle Returned sensor handle
 * @return ESP_OK on success
 */
esp_err_t max31865_init(const max31865_config_t *config, max31865_handle_t *ret_handle);

/**
 * @brief Read temperature from the sensor
 *
 * @param[in] handle Sensor handle
 * @param[out] out_temp Temperature in Celsius
 * @return ESP_OK on success
 */
esp_err_t max31865_read_temp(max31865_handle_t handle, float *out_temp);

/**
 * @brief Deinitialize and free resources
 *
 * @param[in] handle Sensor handle
 * @return ESP_OK on success
 */
esp_err_t max31865_del(max31865_handle_t handle);

#ifdef __cplusplus
}
#endif
