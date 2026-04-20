#include "driver/i2c.h"
#include "driver/gpio.h"
#include "bt_ascii_control.h"
#include "ds3231.h"
#include "hc595_outputs.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max31865.h"
#include "modbus_slave.h"
#include "nvs_flash.h"
#include "rh_sensor.h"
#include "valve_3way.h"
#include <stdint.h>
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

// RLL400 is temporarily disabled because GPIO18 is reassigned to 74HC595 latch.
#define RLL400_ENABLED 0

// MAX31865 Config (HSPI)
#define MAX31865_HOST SPI2_HOST
#define MAX31865_MISO 12
#define MAX31865_MOSI 14
#define MAX31865_CLK 13
#define MAX31865_CS 15
#define MAX31865_CS2 5
#define MAX31865_RREF 1999.0f
#define MAX31865_R0 500.0f

// Valve 3-Way Config
#define VALVE_TEMPERATURE_HYSTERESIS_C 2.5f
#define HEATING_VALVE_NAME "water_rail"
#define HEATING_VALVE_SETPOINT_CHANNEL MODBUS_WATER_CHANNEL_RAIL
#define HEATING_VALVE_PIN_OPEN GPIO_NUM_NC
#define HEATING_VALVE_PIN_CLOSE GPIO_NUM_NC

// 74HC595 control bus
// Assumption: GPIO2 -> SER (data), GPIO4 -> SHCP (clock), GPIO18 -> STCP (latch).
#define HC595_DATA_GPIO GPIO_NUM_2
#define HC595_CLOCK_GPIO GPIO_NUM_4
#define HC595_LATCH_GPIO GPIO_NUM_18

// 74HC595 output mapping
#define HC595_BIT_LIGHT_RELAY_1 0U
#define HC595_BIT_LIGHT_RELAY_2 1U
#define HC595_BIT_HEATING_VALVE_OPEN 2U
#define HC595_BIT_HEATING_VALVE_CLOSE 3U

#define CONTROL_LOOP_MS 200
#define SENSOR_LOOP_MS 5000
#define RTC_INIT_HOUR 10
#define RTC_INIT_MINUTE 31
#define RTC_INIT_SECOND 0

// Static Handles
static rh_sensor_handle_t rh_handle = NULL;
static hc595_outputs_handle_t s_hc595_outputs = NULL;
static max31865_handle_t max_handle = NULL;
static max31865_handle_t max_handle2 = NULL;
static valve_3way_handle_t s_heating_valve = NULL;
static ds3231_handle_t rtc_handle = NULL;
static bool s_rtc_available = false;

static const char *valve_state_to_string(valve_3way_state_t state) {
  switch (state) {
  case VALVE_3WAY_STATE_OPENING:
    return "OPENING";
  case VALVE_3WAY_STATE_CLOSING:
    return "CLOSING";
  case VALVE_3WAY_STATE_STOPPED:
  default:
    return "STOPPED";
  }
}

static esp_err_t init_hc595_outputs(void) {
  hc595_outputs_config_t cfg = {
      .data_gpio_num = HC595_DATA_GPIO,
      .clock_gpio_num = HC595_CLOCK_GPIO,
      .latch_gpio_num = HC595_LATCH_GPIO,
      .relay1_bit_index = HC595_BIT_LIGHT_RELAY_1,
      .relay2_bit_index = HC595_BIT_LIGHT_RELAY_2,
      .valve_open_bit_index = HC595_BIT_HEATING_VALVE_OPEN,
      .valve_close_bit_index = HC595_BIT_HEATING_VALVE_CLOSE,
      .initial_state = 0U,
  };
  return hc595_outputs_init(&cfg, &s_hc595_outputs);
}

static esp_err_t apply_light_outputs(bool relay1_on, bool relay2_on) {
  if (s_hc595_outputs == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  return hc595_outputs_set_light_relays(s_hc595_outputs, relay1_on, relay2_on);
}

static esp_err_t apply_heating_valve_outputs(valve_3way_state_t state) {
  if (s_hc595_outputs == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  hc595_outputs_valve_state_t output_state = HC595_OUTPUTS_VALVE_STOPPED;
  if (state == VALVE_3WAY_STATE_OPENING) {
    output_state = HC595_OUTPUTS_VALVE_OPENING;
  } else if (state == VALVE_3WAY_STATE_CLOSING) {
    output_state = HC595_OUTPUTS_VALVE_CLOSING;
  }
  return hc595_outputs_set_valve_state(s_hc595_outputs, output_state);
}

static esp_err_t init_heating_valve(void) {
  valve_3way_config_t valve_cfg = {
      .name = HEATING_VALVE_NAME,
      .gpio_open_num = HEATING_VALVE_PIN_OPEN,
      .gpio_close_num = HEATING_VALVE_PIN_CLOSE,
      .hysteresis_c = VALVE_TEMPERATURE_HYSTERESIS_C,
      .initial_setpoint_c = 0.0f,
      .initial_actual_temp_c = 0.0f,
  };
  return valve_3way_init(&valve_cfg, &s_heating_valve);
}

static esp_err_t process_heating_valve(float actual_temp_c, bool temp_valid) {
  if (s_heating_valve == NULL) {
    return ESP_ERR_INVALID_STATE;
  }

  if (!temp_valid) {
    esp_err_t err = valve_3way_stop(s_heating_valve);
    if (err != ESP_OK) {
      return err;
    }
    return apply_heating_valve_outputs(valve_3way_get_state(s_heating_valve));
  }

  const float setpoint_c =
      modbus_get_water_setpoint_c(HEATING_VALVE_SETPOINT_CHANNEL);
  esp_err_t err =
      valve_3way_process_temperatures(s_heating_valve, setpoint_c, actual_temp_c);
  if (err != ESP_OK) {
    return err;
  }

  err = apply_heating_valve_outputs(valve_3way_get_state(s_heating_valve));
  if (err != ESP_OK) {
    return err;
  }

  ESP_LOGI(TAG, "Valve %s -> SP: %.2f C | Water: %.2f C | State: %s",
           HEATING_VALVE_NAME, setpoint_c, actual_temp_c,
           valve_state_to_string(valve_3way_get_state(s_heating_valve)));
  return ESP_OK;
}

static bool modbus_rtc_get_time_cb(uint8_t *hour, uint8_t *minute,
                                   uint8_t *second, void *ctx) {
  (void)ctx;
  if (!s_rtc_available || rtc_handle == NULL || hour == NULL || minute == NULL ||
      second == NULL) {
    return false;
  }

  ds3231_time_t rtc_time = {0};
  if (ds3231_get_time(rtc_handle, &rtc_time) != ESP_OK) {
    return false;
  }

  *hour = rtc_time.hour;
  *minute = rtc_time.minute;
  *second = rtc_time.second;
  return true;
}

static bool modbus_rtc_set_time_cb(uint8_t hour, uint8_t minute,
                                   uint8_t second, void *ctx) {
  (void)ctx;
  if (!s_rtc_available || rtc_handle == NULL) {
    return false;
  }

  ds3231_time_t rtc_time = {
      .hour = hour,
      .minute = minute,
      .second = second,
  };
  return ds3231_set_time(rtc_handle, &rtc_time) == ESP_OK;
}

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

static void control_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Control task started (200ms)");

  while (1) {
    bool relay1_on = false;
    bool relay2_on = false;
    modbus_get_light_relay_state(&relay1_on, &relay2_on);
    (void)apply_light_outputs(relay1_on, relay2_on);

    vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
  }
}

static void worker_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Worker task started (5s)");
  TickType_t last_wake = xTaskGetTickCount();

  float rh = 0.0f;
  float temp_air = 0.0f;
  float temp_water_rail = 0.0f;
  bool water_temp_valid = false;

  while (1) {
    // Time source for light schedule: DS3231 when available, otherwise uptime fallback.
    ds3231_time_t rtc_time = {0};
    bool time_ok = false;
    if (s_rtc_available && rtc_handle != NULL) {
      if (ds3231_get_time(rtc_handle, &rtc_time) == ESP_OK) {
        time_ok = true;
      } else {
        ESP_LOGW(TAG, "Failed to read RTC time, using uptime fallback");
      }
    }
    if (!time_ok) {
      uint32_t sec_of_day =
          (uint32_t)((esp_timer_get_time() / 1000000ULL) % 86400ULL);
      rtc_time.hour = (uint8_t)((sec_of_day / 3600U) % 24U);
      rtc_time.minute = (uint8_t)((sec_of_day % 3600U) / 60U);
      rtc_time.second = (uint8_t)(sec_of_day % 60U);
    }
    ESP_LOGI(TAG, "Time source=%s, RTC time=%02u:%02u:%02u",
             time_ok ? "RTC" : "fallback", rtc_time.hour, rtc_time.minute,
             rtc_time.second);
    modbus_set_light_current_time(rtc_time.hour, rtc_time.minute, rtc_time.second);

    float pin_mv = 0.0f;
    float u_sensor = 0.0f;
    if (rh_sensor_read(rh_handle, &pin_mv, &u_sensor, &rh) == ESP_OK) {
      ESP_LOGI(TAG, "RH Sensor -> V_ads: %.0f mV | RH: %.1f %%", pin_mv, rh);
    } else {
      ESP_LOGE(TAG, "Failed to read RH sensor");
    }

    if (max31865_read_temp(max_handle, &temp_air) == ESP_OK) {
      ESP_LOGI(TAG, "PT500 (1) Temp: %.2f C", temp_air);
    } else {
      ESP_LOGE(TAG, "Failed to read PT500 (1)");
    }

    if (max31865_read_temp(max_handle2, &temp_water_rail) == ESP_OK) {
      ESP_LOGI(TAG, "PT500 (2) Temp: %.2f C", temp_water_rail);
      water_temp_valid = true;
    } else {
      ESP_LOGE(TAG, "Failed to read PT500 (2)");
      water_temp_valid = false;
    }

    esp_err_t valve_err = process_heating_valve(temp_water_rail, water_temp_valid);
    if (valve_err != ESP_OK && water_temp_valid) {
      ESP_LOGW(TAG, "Valve %s processing failed: %s", HEATING_VALVE_NAME,
               esp_err_to_name(valve_err));
    }

    float pos = 0.0f;
    float current_ma = 0.0f;
    ESP_LOGI(TAG, "RLL400 disabled -> Pos: %.1f %% | I: %.2f mA", pos,
             current_ma);

    // Telemetry map 0..8 by v2.1:
    // AIR_TEMP=temp_air, AIR_HUM=rh, WATER_RAIL=temp_water_rail,
    // WATER_GROW/UNDERTRAY/UPPER_HEAT currently unavailable -> 0,
    // WINDOWS_POS_A=pos, WINDOWS_POS_B/CURTAIN_POS currently unavailable -> 0.
    modbus_set_telemetry(temp_air, rh, temp_water_rail, 0.0f, 0.0f, 0.0f, pos,
                         0.0f, 0.0f);

    uint8_t light_percent = modbus_get_light_percent();
    ESP_LOGI(TAG, "Light control (relays) -> %u%%", light_percent);

    modbus_mode_state_t mode = modbus_get_mode_state();
    float window_a_target = modbus_get_window_a_target_percent();
    ESP_LOGI(TAG, "Slave mode: %s | Window A target: %.1f%%",
             (mode == MODBUS_MODE_AUTONOMOUS) ? "AUTONOMOUS" : "REMOTE",
             window_a_target);

    ESP_LOGI(TAG, "Heap: %lu (Min: %lu) | Stack HW: %lu",
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size(),
             (unsigned long)uxTaskGetStackHighWaterMark(NULL));

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_LOOP_MS));
  }
}

void app_main(void) {
  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  modbus_init();

  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI(TAG, "I2C initialized");

  ds3231_config_t rtc_cfg = {.i2c_port = I2C_MASTER_NUM, .i2c_addr = DS3231_I2C_ADDR};
  esp_err_t rtc_err = ds3231_init(&rtc_cfg, &rtc_handle);
  if (rtc_err == ESP_OK) {
    s_rtc_available = true;
    ds3231_time_t current_time = {0};
    esp_err_t read_err = ds3231_get_time(rtc_handle, &current_time);
    if (read_err == ESP_OK) {
      ESP_LOGI(TAG, "DS3231 RTC initialized, keep current time %02u:%02u:%02u",
               current_time.hour, current_time.minute, current_time.second);
    } else {
      ds3231_time_t init_time = {
          .hour = RTC_INIT_HOUR,
          .minute = RTC_INIT_MINUTE,
          .second = RTC_INIT_SECOND,
      };
      esp_err_t set_err = ds3231_set_time(rtc_handle, &init_time);
      if (set_err == ESP_OK) {
        ESP_LOGW(TAG,
                 "DS3231 time invalid/unreadable, set fallback time to %02u:%02u:%02u",
                 RTC_INIT_HOUR, RTC_INIT_MINUTE, RTC_INIT_SECOND);
      } else {
        ESP_LOGW(TAG,
                 "DS3231 initialized, but failed to read and set fallback time: %s",
                 esp_err_to_name(set_err));
      }
    }
  } else {
    s_rtc_available = false;
    rtc_handle = NULL;
    ESP_LOGW(TAG, "DS3231 not available (%s), running with uptime fallback time",
             esp_err_to_name(rtc_err));
  }

  modbus_bind_rtc_callbacks(modbus_rtc_get_time_cb, modbus_rtc_set_time_cb,
                            NULL);
  ESP_ERROR_CHECK(bt_ascii_control_init());
  ESP_LOGI(TAG, "Bluetooth ASCII control initialized");
  ESP_ERROR_CHECK(init_hc595_outputs());
  ESP_LOGI(TAG, "74HC595 outputs component initialized");

  rh_sensor_config_t rh_cfg = {.i2c_port = I2C_MASTER_NUM,
                               .i2c_addr = ADS1115_ADDR_GND,
                               .r1_ohm = R1,
                               .r2_ohm = R2,
                               .offset_mv = ADC_OFFSET_MV};
  ESP_ERROR_CHECK(rh_sensor_init(&rh_cfg, &rh_handle));
  ESP_LOGI(TAG, "RH sensor initialized");

  max31865_config_t max_cfg = {.host = MAX31865_HOST,
                               .miso_io_num = MAX31865_MISO,
                               .mosi_io_num = MAX31865_MOSI,
                               .sclk_io_num = MAX31865_CLK,
                               .cs_io_num = MAX31865_CS,
                               .r_ref = MAX31865_RREF,
                               .r0 = MAX31865_R0,
                               .three_wire = true};
  ESP_ERROR_CHECK(max31865_init(&max_cfg, &max_handle));
  ESP_LOGI(TAG, "MAX31865 (1) initialized");

  max31865_config_t max_cfg2 = {.host = MAX31865_HOST,
                                .miso_io_num = MAX31865_MISO,
                                .mosi_io_num = MAX31865_MOSI,
                                .sclk_io_num = MAX31865_CLK,
                                .cs_io_num = MAX31865_CS2,
                                .r_ref = MAX31865_RREF,
                                .r0 = MAX31865_R0,
                                .three_wire = true};
  ESP_ERROR_CHECK(max31865_init(&max_cfg2, &max_handle2));
  ESP_LOGI(TAG, "MAX31865 (2) initialized");

  ESP_ERROR_CHECK(init_heating_valve());
  ESP_ERROR_CHECK(apply_heating_valve_outputs(valve_3way_get_state(s_heating_valve)));
  ESP_ERROR_CHECK(apply_light_outputs(false, false));

#if !RLL400_ENABLED
  ESP_LOGW(TAG, "RLL400 control is disabled while GPIO18 is used by 74HC595");
#endif

  xTaskCreate(control_task, "ctrl", 4096, NULL, 6, NULL);
  xTaskCreate(worker_task, "worker", 8192, NULL, 5, NULL);
}
