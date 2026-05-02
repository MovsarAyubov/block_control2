#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "ds3231.h"
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
#include "rll400.h"
#include "valve_3way.h"
#include "sdkconfig.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#define MAX31865_CS2 5
#define MAX31865_RREF 1999.0f
#define MAX31865_R0 500.0f

// Valve 3-Way Config
#define VALVE_PIN_OPEN 32
#define VALVE_PIN_CLOSE 33
#define VALVE_TEMP_OPEN 18.0f
#define VALVE_TEMP_CLOSE 25.0f

// Light relays
#define LIGHT_RELAY_1_GPIO 4
#define LIGHT_RELAY_2_GPIO 16

#define CONTROL_LOOP_MS 200
#define SENSOR_LOOP_MS 5000
#define RTC_INIT_HOUR 10
#define RTC_INIT_MINUTE 31
#define RTC_INIT_SECOND 0
#define USB_BRIDGE_UART UART_NUM_0
#define USB_BRIDGE_BAUD 115200
#define USB_BRIDGE_LINE_MAX 192
#define USB_BRIDGE_REG_MAX 64

// Static Handles
static rh_sensor_handle_t rh_handle = NULL;
static rll400_handle_t rll_handle = NULL;
static max31865_handle_t max_handle = NULL;
static max31865_handle_t max_handle2 = NULL;
static valve_3way_handle_t valve_handle = NULL;
static ds3231_handle_t rtc_handle = NULL;
static bool s_rtc_available = false;

static volatile float s_pos_pct = 0.0f;
static volatile float s_current_ma = 0.0f;

#if CONFIG_GREENHOUSE_USB_BRIDGE
static void usb_bridge_write(const char *text) {
  if (text != NULL) {
    (void)uart_write_bytes(USB_BRIDGE_UART, text, strlen(text));
  }
}

static void usb_bridge_handle_line(char *line) {
  char *save = NULL;
  char *cmd = strtok_r(line, " \t\r\n", &save);
  if (cmd == NULL || strcmp(cmd, "GH") != 0) {
    return;
  }

  char *op = strtok_r(NULL, " \t\r\n", &save);
  if (op == NULL) {
    usb_bridge_write("GH ERR missing-op\n");
    return;
  }

  if (strcmp(op, "PING") == 0) {
    usb_bridge_write("GH OK PONG\n");
    return;
  }

  if (strcmp(op, "READ") == 0) {
    char *start_s = strtok_r(NULL, " \t\r\n", &save);
    char *count_s = strtok_r(NULL, " \t\r\n", &save);
    if (start_s == NULL || count_s == NULL) {
      usb_bridge_write("GH ERR read-args\n");
      return;
    }
    uint16_t start = (uint16_t)strtoul(start_s, NULL, 0);
    uint16_t count = (uint16_t)strtoul(count_s, NULL, 0);
    if (count == 0U || count > USB_BRIDGE_REG_MAX) {
      usb_bridge_write("GH ERR read-count\n");
      return;
    }
    uint16_t regs[USB_BRIDGE_REG_MAX] = {0};
    if (!modbus_read_holding_regs(start, count, regs)) {
      usb_bridge_write("GH ERR read-range\n");
      return;
    }
    char out[USB_BRIDGE_LINE_MAX] = {0};
    int used = snprintf(out, sizeof(out), "GH DATA %u %u", (unsigned)start,
                        (unsigned)count);
    for (uint16_t i = 0; i < count && used > 0 && used < (int)sizeof(out); ++i) {
      used += snprintf(out + used, sizeof(out) - (size_t)used, " %u",
                       (unsigned)regs[i]);
    }
    if (used > 0 && used < (int)sizeof(out)) {
      snprintf(out + used, sizeof(out) - (size_t)used, "\n");
    } else {
      out[sizeof(out) - 2U] = '\n';
      out[sizeof(out) - 1U] = '\0';
    }
    usb_bridge_write(out);
    return;
  }

  if (strcmp(op, "WRITE") == 0) {
    char *start_s = strtok_r(NULL, " \t\r\n", &save);
    if (start_s == NULL) {
      usb_bridge_write("GH ERR write-args\n");
      return;
    }
    uint16_t start = (uint16_t)strtoul(start_s, NULL, 0);
    uint16_t values[USB_BRIDGE_REG_MAX] = {0};
    uint16_t count = 0;
    char *value_s = NULL;
    while ((value_s = strtok_r(NULL, " \t\r\n", &save)) != NULL &&
           count < USB_BRIDGE_REG_MAX) {
      values[count++] = (uint16_t)strtoul(value_s, NULL, 0);
    }
    if (count == 0U) {
      usb_bridge_write("GH ERR write-count\n");
      return;
    }
    if (!modbus_write_holding_regs(start, values, count)) {
      usb_bridge_write("GH ERR write-range\n");
      return;
    }
    char out[64] = {0};
    snprintf(out, sizeof(out), "GH OK WRITE %u\n", (unsigned)count);
    usb_bridge_write(out);
    return;
  }

  usb_bridge_write("GH ERR unknown-op\n");
}

static void usb_bridge_task(void *arg) {
  (void)arg;
  uart_config_t cfg = {
      .baud_rate = USB_BRIDGE_BAUD,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };
  (void)uart_param_config(USB_BRIDGE_UART, &cfg);
  esp_err_t install_err =
      uart_driver_install(USB_BRIDGE_UART, 2048, 2048, 0, NULL, 0);
  if (install_err != ESP_OK && install_err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "USB bridge UART install failed: %s",
             esp_err_to_name(install_err));
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "USB bridge enabled on UART0 (%u baud)", USB_BRIDGE_BAUD);
  usb_bridge_write("GH READY block_control2 usb-bridge\n");

  char line[USB_BRIDGE_LINE_MAX] = {0};
  size_t len = 0;
  uint8_t byte = 0;
  while (1) {
    int got = uart_read_bytes(USB_BRIDGE_UART, &byte, 1, pdMS_TO_TICKS(100));
    if (got <= 0) {
      continue;
    }
    if (byte == '\n' || byte == '\r') {
      if (len > 0) {
        line[len] = '\0';
        usb_bridge_handle_line(line);
        len = 0;
      }
      continue;
    }
    if (len + 1 < sizeof(line)) {
      line[len++] = (char)byte;
    } else {
      len = 0;
      usb_bridge_write("GH ERR line-too-long\n");
    }
  }
}
#endif

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

static void light_relay_init(void) {
  gpio_config_t io_conf = {
      .pin_bit_mask = (1ULL << LIGHT_RELAY_1_GPIO) | (1ULL << LIGHT_RELAY_2_GPIO),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_set_level(LIGHT_RELAY_1_GPIO, 0));
  ESP_ERROR_CHECK(gpio_set_level(LIGHT_RELAY_2_GPIO, 0));
}

static void control_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Control task started (200ms)");

  while (1) {
    float target_pct = modbus_get_window_a_target_percent();
    (void)rll400_set_target(rll_handle, target_pct);

    (void)rll400_process(rll_handle);

    float pos = 0.0f;
    float current = 0.0f;
    if (rll400_get_status(rll_handle, &pos, &current) == ESP_OK) {
      s_pos_pct = pos;
      s_current_ma = current;
    }

    bool relay1_on = false;
    bool relay2_on = false;
    modbus_get_light_relay_state(&relay1_on, &relay2_on);
    (void)gpio_set_level(LIGHT_RELAY_1_GPIO, relay1_on ? 1 : 0);
    (void)gpio_set_level(LIGHT_RELAY_2_GPIO, relay2_on ? 1 : 0);

    vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
  }
}

static bool is_plausible_process_temp(float value) {
  return value >= -40.0f && value <= 120.0f;
}

static void worker_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Worker task started (5s)");
  TickType_t last_wake = xTaskGetTickCount();

  float rh = 0.0f;
  float temp_air = 22.0f;
  float temp_water_rail = 24.0f;

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

    float read_temp_air = 0.0f;
    if (max31865_read_temp(max_handle, &read_temp_air) == ESP_OK &&
        is_plausible_process_temp(read_temp_air)) {
      temp_air = read_temp_air;
      ESP_LOGI(TAG, "PT500 (1) Temp: %.2f C", temp_air);
      (void)valve_3way_process(valve_handle, temp_air);
    } else {
      ESP_LOGE(TAG, "Invalid PT500 (1) read, keeping last air temp: %.2f C",
               temp_air);
    }

    float read_temp_water_rail = 0.0f;
    if (max31865_read_temp(max_handle2, &read_temp_water_rail) == ESP_OK &&
        is_plausible_process_temp(read_temp_water_rail)) {
      temp_water_rail = read_temp_water_rail;
      ESP_LOGI(TAG, "PT500 (2) Temp: %.2f C", temp_water_rail);
    } else {
      ESP_LOGE(TAG,
               "Invalid PT500 (2) read, keeping last water rail temp: %.2f C",
               temp_water_rail);
    }

    float pos = s_pos_pct;
    float current_ma = s_current_ma;
    ESP_LOGI(TAG, "RLL400 -> Pos: %.1f %% | I: %.2f mA", pos, current_ma);

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

#if CONFIG_GREENHOUSE_DEVICE_MODE_SIMULATED
static void simulated_control_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Simulated control task started");

  while (1) {
    bool relay1_on = false;
    bool relay2_on = false;
    modbus_get_light_relay_state(&relay1_on, &relay2_on);
    ESP_LOGI(TAG, "SIM light relays -> relay1=%u relay2=%u",
             relay1_on ? 1U : 0U, relay2_on ? 1U : 0U);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void simulated_worker_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "Simulated worker task started");

  while (1) {
    uint32_t sec = (uint32_t)((esp_timer_get_time() / 1000000ULL) % 86400ULL);
    uint8_t hour = (uint8_t)((sec / 3600U) % 24U);
    uint8_t minute = (uint8_t)((sec % 3600U) / 60U);
    uint8_t second = (uint8_t)(sec % 60U);
    modbus_set_light_current_time(hour, minute, second);

    float t = (float)(esp_timer_get_time() / 1000000ULL);
    float temp_air = 22.0f + sinf(t / 60.0f) * 2.0f;
    float rh = 60.0f + cosf(t / 45.0f) * 8.0f;
    float water = 24.0f + sinf(t / 50.0f) * 1.5f;
    float pos = 20.0f + sinf(t / 80.0f) * 15.0f;

    modbus_set_telemetry(temp_air, rh, water, water - 0.5f, water - 1.0f,
                         water + 2.0f, pos, pos * 0.9f, 0.0f);
    modbus_set_solar_radiation(320.0f + sinf(t / 30.0f) * 180.0f);

    ESP_LOGI(TAG, "SIM telemetry -> air=%.1fC rh=%.1f%% water=%.1fC pos=%.1f%%",
             temp_air, rh, water, pos);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_LOOP_MS));
  }
}
#endif

void app_main(void) {
#if CONFIG_GREENHOUSE_DEVICE_MODE_PHYSICAL
  light_relay_init();
#endif

  esp_err_t nvs_err = nvs_flash_init();
  if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(nvs_err);

  modbus_init();
#if CONFIG_GREENHOUSE_USB_BRIDGE
  xTaskCreate(usb_bridge_task, "usb_bridge", 4096, NULL, 7, NULL);
#endif

#if CONFIG_GREENHOUSE_DEVICE_MODE_SIMULATED
  ESP_LOGW(TAG, "Greenhouse device mode: SIMULATED");
  xTaskCreate(simulated_control_task, "sim_ctrl", 4096, NULL, 6, NULL);
  xTaskCreate(simulated_worker_task, "sim_worker", 4096, NULL, 5, NULL);
  return;
#else
  ESP_LOGI(TAG, "Greenhouse device mode: PHYSICAL");

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

  rh_sensor_config_t rh_cfg = {.i2c_port = I2C_MASTER_NUM,
                               .i2c_addr = ADS1115_ADDR_GND,
                               .r1_ohm = R1,
                               .r2_ohm = R2,
                               .offset_mv = ADC_OFFSET_MV};
  ESP_ERROR_CHECK(rh_sensor_init(&rh_cfg, &rh_handle));
  ESP_LOGI(TAG, "RH sensor initialized");

  rll400_config_t rll_cfg = {.i2c_port = I2C_MASTER_NUM,
                             .ads_addr = ADS1115_ADDR_GND,
                             .shunt_resistor_ohm = RLL_SHUNT_OHM,
                             .pin_open = RLL_PIN_OPEN,
                             .pin_close = RLL_PIN_CLOSE,
                             .hysteresis_percent = 2.0f};
  ESP_ERROR_CHECK(rll400_init(&rll_cfg, &rll_handle));
  ESP_LOGI(TAG, "RLL400 initialized");

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

  valve_3way_config_t valve_cfg = {.gpio_open_num = VALVE_PIN_OPEN,
                                   .gpio_close_num = VALVE_PIN_CLOSE,
                                   .temp_open_threshold = VALVE_TEMP_OPEN,
                                   .temp_close_threshold = VALVE_TEMP_CLOSE};
  ESP_ERROR_CHECK(valve_3way_init(&valve_cfg, &valve_handle));

  xTaskCreate(control_task, "ctrl", 4096, NULL, 6, NULL);
  xTaskCreate(worker_task, "worker", 8192, NULL, 5, NULL);
#endif
}
