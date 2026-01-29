#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rh_sensor.h"
#include "rll400.h"
#include <stdio.h>

static const char *TAG = "APP";

// I2C Configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// Sensor Config
#define R1 5220.0f
#define R2 10000.0f
#define ADC_OFFSET_MV 0

// RLL400 Config
#define RLL_SHUNT_OHM 109.9f
#define RLL_PIN_OPEN 18
#define RLL_PIN_CLOSE 19

static esp_err_t i2c_master_init(void) {
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void) {
  // 1. Init I2C
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C Initialized");

  // 2. Initialize RH Sensor
  rh_sensor_config_t rh_cfg = {.i2c_port = I2C_MASTER_NUM,
                               .i2c_addr = ADS1115_ADDR_GND, // 0x48
                               .r1_ohm = R1,
                               .r2_ohm = R2,
                               .offset_mv = ADC_OFFSET_MV};

  rh_sensor_handle_t rh_handle = NULL;
  ESP_ERROR_CHECK(rh_sensor_init(&rh_cfg, &rh_handle));
  ESP_LOGI(TAG, "RH Sensor Initialized");

  // 3. Initialize RLL400
  rll400_config_t rll_cfg = {.i2c_port = I2C_MASTER_NUM,
                             .ads_addr = ADS1115_ADDR_GND,
                             .shunt_resistor_ohm = RLL_SHUNT_OHM,
                             .pin_open = RLL_PIN_OPEN,
                             .pin_close = RLL_PIN_CLOSE,
                             .hysteresis_percent = 2.0f};

  rll400_handle_t rll_handle = NULL;
  ESP_ERROR_CHECK(rll400_init(&rll_cfg, &rll_handle));
  ESP_LOGI(TAG, "RLL400 Initialized");

  // Demo: Set target to 50%
  rll400_set_target(rll_handle, 50.0f);

  while (1) {
    // Read RH
    float pin_mv, u_sensor, rh;
    if (rh_sensor_read(rh_handle, &pin_mv, &u_sensor, &rh) == ESP_OK) {
      ESP_LOGI(TAG, "RH Sensor -> V_ads: %.0f mV | RH: %.1f %%", pin_mv, rh);
    } else {
      ESP_LOGE(TAG, "Failed to read RH sensor");
    }

    // Process RLL400 Control Loop
    rll400_process(rll_handle);

    float pos_pct, current_ma;
    if (rll400_get_status(rll_handle, &pos_pct, &current_ma) == ESP_OK) {
      ESP_LOGI(TAG, "RLL400 -> Pos: %.1f %% | I: %.2f mA", pos_pct, current_ma);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}