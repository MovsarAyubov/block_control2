#include "rh_sensor.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>


static const char *TAG = "rh_sensor";

// ADS1115 Registers
#define ADS_REG_CONV 0x00
#define ADS_REG_CFG 0x01

// ADS1115 Config Field Shifts
#define ADS_CFG_OS_SINGLE (0x8000)
#define ADS_CFG_MUX_OFFSET (12)
#define ADS_CFG_PGA_4_096V (0x0200)  // Gain=1 (+/-4.096V)
#define ADS_CFG_MODE_SINGLE (0x0100) // Single-shot mode
#define ADS_CFG_DR_128SPS (0x0080)   // 128 SPS
#define ADS_CFG_COMP_QUE_DISABLE (0x0003)

typedef struct rh_sensor_ctx_t {
  rh_sensor_config_t config;
  float divider_ratio;
} rh_sensor_ctx_t;

static esp_err_t _ads1115_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg,
                                    uint16_t value) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true); // MSB
  i2c_master_write_byte(cmd, value & 0xFF, true);        // LSB
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);
  return ret;
}

static esp_err_t _ads1115_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg,
                                   int16_t *val) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);

  // Restart for read
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
  uint8_t msb, lsb;
  i2c_master_read_byte(cmd, &msb, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &lsb, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK) {
    *val = (int16_t)((msb << 8) | lsb);
  }
  return ret;
}

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

  *ret_handle = ctx;
  return ESP_OK;
}

esp_err_t rh_sensor_read(rh_sensor_handle_t handle, float *out_pin_mv,
                         float *out_sensor_v, float *out_rh) {
  ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
  rh_sensor_ctx_t *ctx = handle;

  // 1. Configure ADS1115 for Single Shot on correct channel
  // Mux: 100xxxxx for AINp=AIN0, AINn=GND (if chan=0)
  // Map channel 0->4, 1->5, 2->6, 3->7 for MUX bits (Single ended)
  uint16_t mux = (0x4 + ctx->config.ads_channel) << ADS_CFG_MUX_OFFSET;

  uint16_t config = ADS_CFG_OS_SINGLE | mux | ADS_CFG_PGA_4_096V |
                    ADS_CFG_MODE_SINGLE | ADS_CFG_DR_128SPS |
                    ADS_CFG_COMP_QUE_DISABLE;

  ESP_RETURN_ON_ERROR(_ads1115_write_reg(ctx->config.i2c_port,
                                         ctx->config.i2c_addr, ADS_REG_CFG,
                                         config),
                      TAG, "Failed to write config");

  // 2. Wait for conversion (rough estimate ~8ms for 128SPS, add margin)
  vTaskDelay(pdMS_TO_TICKS(20));

  // 3. Read Result
  int16_t raw_val = 0;
  ESP_RETURN_ON_ERROR(_ads1115_read_reg(ctx->config.i2c_port,
                                        ctx->config.i2c_addr, ADS_REG_CONV,
                                        &raw_val),
                      TAG, "Failed to read conv");

  // 4. Convert to Voltage
  // PGA=1 => +/- 4.096V. 1 bit = 4.096 / 32768 = 0.125mV
  float voltage_mv = raw_val * 0.125f;

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
  if (handle)
    free(handle);
  return ESP_OK;
}
