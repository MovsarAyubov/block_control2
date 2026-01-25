#include "rh_sensor.h"
#include "ads1115.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "rh_sensor";

typedef struct rh_sensor_ctx_t {
  rh_sensor_config_t config;
  float divider_ratio;
  ads1115_handle_t ads_handle;
} rh_sensor_ctx_t;

esp_err_t rh_sensor_init(const rh_sensor_config_t *config,
                         rh_sensor_handle_t *ret_handle) {
  ESP_RETURN_ON_FALSE(config && ret_handle, ESP_ERR_INVALID_ARG, TAG,
                      "Invalid arguments");

  rh_sensor_ctx_t *ctx = calloc(1, sizeof(rh_sensor_ctx_t));
  if (!ctx)
    return ESP_ERR_NO_MEM;

  memcpy(&ctx->config, config, sizeof(rh_sensor_config_t));

  // Divider K = R2 / (R1 + R2)
  if ((ctx->config.r1_ohm + ctx->config.r2_ohm) > 0) {
    ctx->divider_ratio =
        ctx->config.r2_ohm / (ctx->config.r1_ohm + ctx->config.r2_ohm);
  } else {
    ctx->divider_ratio = 1.0f;
  }

  // Initialize ADS1115 driver
  ads1115_config_t ads_cfg = {
      .i2c_port = config->i2c_port,
      .i2c_addr = config->i2c_addr,
  };

  esp_err_t err = ads1115_init(&ads_cfg, &ctx->ads_handle);
  if (err != ESP_OK) {
    free(ctx);
    return err;
  }

  *ret_handle = ctx;
  return ESP_OK;
}

esp_err_t rh_sensor_read(rh_sensor_handle_t handle, float *out_pin_mv,
                         float *out_sensor_v, float *out_rh) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  rh_sensor_ctx_t *ctx = handle;

  // 1. Read Voltage (mV) using shared driver
  float voltage_mv = 0;
  ESP_RETURN_ON_ERROR(ads1115_read_voltage(ctx->ads_handle,
                                           ctx->config.ads_channel,
                                           &voltage_mv),
                      TAG, "Failed to read voltage");

  // Apply Offset
  voltage_mv += ctx->config.offset_mv;
  if (voltage_mv < 0)
    voltage_mv = 0;

  // Calc Logic
  float v_adc_volts = voltage_mv / 1000.0f;
  float u_out = v_adc_volts / ctx->divider_ratio;

  // Formula: RH = 20 * U_out + 12.5
  float rh = 20.0f * u_out + 12.5f;

  // Clamp
  if (rh < 0.0f)
    rh = 0.0f;
  if (rh > 100.0f)
    rh = 100.0f;

  if (out_pin_mv)
    *out_pin_mv = voltage_mv;
  if (out_sensor_v)
    *out_sensor_v = u_out;
  if (out_rh)
    *out_rh = rh;

  return ESP_OK;
}

esp_err_t rh_sensor_del(rh_sensor_handle_t handle) {
  if (handle) {
    rh_sensor_ctx_t *ctx = handle;
    if (ctx->ads_handle) {
      ads1115_del(ctx->ads_handle);
    }
    free(ctx);
  }
  return ESP_OK;
}
