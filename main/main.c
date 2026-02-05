#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max31865.h"
#include "rh_sensor.h"
#include "rll400.h"
#include "valve_3way.h"
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

// MAX31865 Config (HSPI)
#define MAX31865_HOST SPI2_HOST
#define MAX31865_MISO 12
#define MAX31865_MOSI 13
#define MAX31865_CLK 14
#define MAX31865_CS 15
#define MAX31865_CS2 5  // Второй датчик
#define MAX31865_RREF 1999.0f
#define MAX31865_R0 500.0f

// Valve 3-Way Config
#define VALVE_PIN_OPEN 26
#define VALVE_PIN_CLOSE 27
#define VALVE_TEMP_OPEN 18.0f
#define VALVE_TEMP_CLOSE 25.0f

// LED Status Config
#define LED_GPIO 4
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO LED_GPIO
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY 5000
#define LEDC_MAX_DUTY 8191 // 2^13 - 1

// Static Handles
static rh_sensor_handle_t rh_handle = NULL;
static rll400_handle_t rll_handle = NULL;
static max31865_handle_t max_handle = NULL;
static max31865_handle_t max_handle2 = NULL;  // Второй датчик
static valve_3way_handle_t valve_handle = NULL;

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

static void led_init(void) {
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_MODE,
                                    .timer_num = LEDC_TIMER,
                                    .duty_resolution = LEDC_DUTY_RES,
                                    .freq_hz = LEDC_FREQUENCY,
                                    .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t ledc_channel = {.speed_mode = LEDC_MODE,
                                        .channel = LEDC_CHANNEL,
                                        .timer_sel = LEDC_TIMER,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .gpio_num = LEDC_OUTPUT_IO,
                                        .duty = 0, // Start Off
                                        .hpoint = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  // Initialize fade service
  ledc_fade_func_install(0);
}

void worker_task(void *arg) {
  ESP_LOGI(TAG, "Worker Task Started");

  while (1) {
    // Start of cycle: Fade out
    ledc_set_fade_with_time(LEDC_MODE, LEDC_CHANNEL, 0, 500);
    ledc_fade_start(LEDC_MODE, LEDC_CHANNEL, LEDC_FADE_NO_WAIT);

    // Read RH
    float pin_mv, u_sensor, rh;
    if (rh_sensor_read(rh_handle, &pin_mv, &u_sensor, &rh) == ESP_OK) {
      ESP_LOGI(TAG, "RH Sensor -> V_ads: %.0f mV | RH: %.1f %%", pin_mv, rh);
    } else {
      ESP_LOGE(TAG, "Failed to read RH sensor");
    }

    // Process RLL400 Control Loop
    // ESP_LOGI(TAG, "Dbg: Pre RLL Process");
    rll400_process(rll_handle);
    // ESP_LOGI(TAG, "Dbg: Post RLL Process");

    float pos_pct, current_ma;
    if (rll400_get_status(rll_handle, &pos_pct, &current_ma) == ESP_OK) {
      // Use snprintf to ensure formatting safety
      // char buf[64];
      // snprintf(buf, sizeof(buf), "RLL400 -> Pos: %.1f %% | I: %.2f mA",
      // pos_pct, current_ma); ESP_LOGI(TAG, "%s", buf);
      ESP_LOGI(TAG, "RLL400 -> Pos: %.1f %% | I: %.2f mA", pos_pct, current_ma);
    }
    // ESP_LOGI(TAG, "Dbg: Post RLL Status");

    // Read PT500 Temp
    float temp_c;
    if (max31865_read_temp(max_handle, &temp_c) == ESP_OK) {
      ESP_LOGI(TAG, "PT500 (1) Temp: %.2f C", temp_c);

      // Process Valve Control
      valve_3way_process(valve_handle, temp_c);

    } else {
      ESP_LOGE(TAG, "Failed to read PT500 (1)");
    }

    // Read PT500 (2) Temp
    float temp_c2;
    if (max31865_read_temp(max_handle2, &temp_c2) == ESP_OK) {
      ESP_LOGI(TAG, "PT500 (2) Temp: %.2f C", temp_c2);
    } else {
      ESP_LOGE(TAG, "Failed to read PT500 (2)");
    }

    // End of cycle: Turn ON
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_MAX_DUTY);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Debug: Monitor Heap and Stack
    ESP_LOGI(TAG, "Heap: %lu (Min: %lu) | Stack HW: %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size(),
             (unsigned long)uxTaskGetStackHighWaterMark(NULL));
  }
}

void app_main(void) {
  // 0. Init LED
  led_init();

  // 1. Init I2C
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C Initialized");

  // 2. Initialize RH Sensor
  rh_sensor_config_t rh_cfg = {.i2c_port = I2C_MASTER_NUM,
                               .i2c_addr = ADS1115_ADDR_GND, // 0x48
                               .r1_ohm = R1,
                               .r2_ohm = R2,
                               .offset_mv = ADC_OFFSET_MV};

  ESP_ERROR_CHECK(rh_sensor_init(&rh_cfg, &rh_handle));
  ESP_LOGI(TAG, "RH Sensor Initialized");

  // 3. Initialize RLL400
  rll400_config_t rll_cfg = {.i2c_port = I2C_MASTER_NUM,
                             .ads_addr = ADS1115_ADDR_GND,
                             .shunt_resistor_ohm = RLL_SHUNT_OHM,
                             .pin_open = RLL_PIN_OPEN,
                             .pin_close = RLL_PIN_CLOSE,
                             .hysteresis_percent = 2.0f};

  ESP_ERROR_CHECK(rll400_init(&rll_cfg, &rll_handle));
  ESP_LOGI(TAG, "RLL400 Initialized");

  // Demo: Set target to 50%
  rll400_set_target(rll_handle, 50.0f);

  // 4. Initialize MAX31865 (PT500)
  max31865_config_t max_cfg = {.host = MAX31865_HOST,
                               .miso_io_num = MAX31865_MISO,
                               .mosi_io_num = MAX31865_MOSI,
                               .sclk_io_num = MAX31865_CLK,
                               .cs_io_num = MAX31865_CS,
                               .r_ref = MAX31865_RREF,
                               .r0 = MAX31865_R0,
                               .three_wire = true};
  ESP_ERROR_CHECK(max31865_init(&max_cfg, &max_handle));
  ESP_LOGI(TAG, "MAX31865 (1) Initialized");

  // Initialize MAX31865 (2) - Второй датчик PT500
  max31865_config_t max_cfg2 = {.host = MAX31865_HOST,
                                .miso_io_num = MAX31865_MISO,
                                .mosi_io_num = MAX31865_MOSI,
                                .sclk_io_num = MAX31865_CLK,
                                .cs_io_num = MAX31865_CS2,
                                .r_ref = MAX31865_RREF,
                                .r0 = MAX31865_R0,
                                .three_wire = true};
  ESP_ERROR_CHECK(max31865_init(&max_cfg2, &max_handle2));
  ESP_LOGI(TAG, "MAX31865 (2) Initialized");

  // 5. Initialize Valve 3-Way
  valve_3way_config_t valve_cfg = {.gpio_open_num = VALVE_PIN_OPEN,
                                   .gpio_close_num = VALVE_PIN_CLOSE,
                                   .temp_open_threshold = VALVE_TEMP_OPEN,
                                   .temp_close_threshold = VALVE_TEMP_CLOSE};
  ESP_ERROR_CHECK(valve_3way_init(&valve_cfg, &valve_handle));

  // 6. Create Worker Task
  xTaskCreate(worker_task, "worker", 8192, NULL, 5, NULL);
}
