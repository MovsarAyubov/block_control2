#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rh_sensor.h"
#include <stdio.h>

static const char *TAG = "APP";

// I2C Configuration
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// User Hardware Setup
#define R1 5220.0f
#define R2 10000.0f
#define ADC_OFFSET_MV 0 // ADS1115 is usually precise, start with 0 offset

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

  // 2. Initialize Sensor (ADS1115)
  rh_sensor_config_t config = {.i2c_port = I2C_MASTER_NUM,
                               .i2c_addr = ADS1115_ADDR_GND, // 0x48
                               .ads_channel = 0,             // AIN0
                               .r1_ohm = R1,
                               .r2_ohm = R2,
                               .offset_mv = ADC_OFFSET_MV};

  rh_sensor_handle_t sensor_handle = NULL;
  ESP_ERROR_CHECK(rh_sensor_init(&config, &sensor_handle));

  ESP_LOGI(TAG, "Sensor Initialized (ADS1115)");

  while (1) {
    float pin_mv, u_sensor, rh;

    // 3. Read Sensor
    esp_err_t err = rh_sensor_read(sensor_handle, &pin_mv, &u_sensor, &rh);

    if (err == ESP_OK) {
      ESP_LOGI(TAG, "V_ads: %.0f mV | U_sensor: %.3f V | RH: %.1f %%", pin_mv,
               u_sensor, rh);
    } else {
      ESP_LOGE(TAG, "Failed to read sensor (Check wiring/I2C)");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}