#include "max31865.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "MAX31865";

// MAX31865 Registers
#define MAX31865_CONFIG_REG 0x00
#define MAX31865_RTD_MSB_REG 0x01
#define MAX31865_RTD_LSB_REG 0x02
#define MAX31865_HIGH_FAULT_THRESH_MSB_REG 0x03
#define MAX31865_HIGH_FAULT_THRESH_LSB_REG 0x04
#define MAX31865_LOW_FAULT_THRESH_MSB_REG 0x05
#define MAX31865_LOW_FAULT_THRESH_LSB_REG 0x06
#define MAX31865_FAULT_STATUS_REG 0x07

// Configuration Bits
#define MAX31865_CONFIG_BIAS 0x80
#define MAX31865_CONFIG_MODEAUTO 0x40
#define MAX31865_CONFIG_MODEOFF 0x00
#define MAX31865_CONFIG_1SHOT 0x20
#define MAX31865_CONFIG_3WIRE 0x10
#define MAX31865_CONFIG_24WIRE 0x00
#define MAX31865_CONFIG_FAULTSTAT 0x02
#define MAX31865_CONFIG_FILT50HZ 0x01
#define MAX31865_CONFIG_FILT60HZ 0x00

// PT500 Constants (Callendar-Van Dusen equation)
#define RTD_A 3.9083e-3f
#define RTD_B -5.775e-7f

struct max31865_ctx_t {
  spi_device_handle_t spi;
  max31865_config_t cfg;
};

static esp_err_t write_reg(spi_device_handle_t spi, uint8_t addr,
                           uint8_t data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = 8;
  t.addr = (addr | 0x80); // Set write bit (0x80)
  t.cmd = 0;              // Not using command phase
  t.tx_buffer = &data;
  // We are not using addr phase in standard sense, but MAX31865 expects Address
  // byte first. Address byte: [Write=1][Address 7 bits] Wait, typical SPI
  // devices: First byte is address (with R/W bit), subsequent bytes data. Let's
  // implement manually by sending 2 bytes: [Address][Data]

  uint8_t tx_data[2] = {addr | 0x80, data}; // 0x80 for Write
  t.length = 16;
  t.tx_buffer = tx_data;

  ret = spi_device_transmit(spi, &t);
  return ret;
}

static esp_err_t read_reg(spi_device_handle_t spi, uint8_t addr,
                          uint8_t *data) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  uint8_t tx_data[2] = {addr & 0x7F, 0x00}; // 0x7F mask for Read (MSB 0)
  uint8_t rx_data[2] = {0};

  t.length = 16;
  t.tx_buffer = tx_data;
  t.rx_buffer = rx_data;

  ret = spi_device_transmit(spi, &t);
  *data = rx_data[1]; // Second byte is the data
  return ret;
}

static esp_err_t read_regs(spi_device_handle_t spi, uint8_t addr, uint8_t *data,
                           size_t len) {
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));

  if (len > 4) {
    // Fallback for larger reads (not expected in this driver usually)
    ESP_LOGE(TAG, "read_regs len > 4 not supported in optimized version");
    return ESP_ERR_NOT_SUPPORTED;
  }

  // Use internal buffers (avoid malloc)
  t.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
  t.length = (len + 1) * 8;

  // First byte is address, rest are dummies
  t.tx_data[0] = addr & 0x7F; // Read
  // No need to memset dummy bytes in tx_data as MOSI is ignored during read
  // phase usually, but good practice to zero or set them. The memset 0 in
  // struct init handled it.

  ret = spi_device_transmit(spi, &t);

  // Copy received data (skip first byte which is response to address/dummy)
  // Wait, standard SPI behavior:
  // Cycle 0: MASTER sends ADDR, SLAVE sends ??? (HiZ or garbage)
  // Cycle 1..N: MASTER sends Dummy, SLAVE sends DATA
  // So rx_data[0] corresponds to ADDR phase, rx_data[1] is first data byte.
  if (ret == ESP_OK) {
    memcpy(data, &t.rx_data[1], len);
  }

  return ret;
}

esp_err_t max31865_init(const max31865_config_t *config,
                        max31865_handle_t *ret_handle) {
  if (!config || !ret_handle)
    return ESP_ERR_INVALID_ARG;

  struct max31865_ctx_t *ctx = calloc(1, sizeof(struct max31865_ctx_t));
  if (!ctx)
    return ESP_ERR_NO_MEM;

  ctx->cfg = *config;

  // Initialize SPI Bus (if needed) - We assume user might initialize bus
  // elsewhere, but here we can try to initialize. If it fails (already
  // initialized), we ignore. However, best practice is user initializes bus.
  // But given the prompt context, we should robustly handle bus initialization
  // or assume it. Since we are using HSPI, let's initialize it here if we can.

  spi_bus_config_t buscfg = {
      .miso_io_num = config->miso_io_num,
      .mosi_io_num = config->mosi_io_num,
      .sclk_io_num = config->sclk_io_num,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 32,
  };

  // Try to initialize bus. If it returns ESP_ERR_INVALID_STATE, it means
  // already initialized.
  esp_err_t ret = spi_bus_initialize(config->host, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus");
    free(ctx);
    return ret;
  }

  // Add device to bus
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 1000000, // 1 MHz
      .mode = 1, // CPOL=0, CPHA=1 (MAX31865 supports Mode 1 and 3)
      .spics_io_num = config->cs_io_num,
      .queue_size = 1,
  };

  ret = spi_bus_add_device(config->host, &devcfg, &ctx->spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add device to SPI bus");
    free(ctx);
    return ret;
  }

  // Configure MAX31865
  uint8_t config_byte = MAX31865_CONFIG_BIAS | MAX31865_CONFIG_FAULTSTAT |
                        MAX31865_CONFIG_FILT50HZ;
  if (config->three_wire) {
    config_byte |= MAX31865_CONFIG_3WIRE;
  }

  // Write configuration
  write_reg(ctx->spi, MAX31865_CONFIG_REG, config_byte);

  *ret_handle = ctx;
  ESP_LOGI(TAG, "Initialized MAX31865 (Host: %d, CS: %d, 3-Wire: %d)",
           config->host, config->cs_io_num, config->three_wire);

  return ESP_OK;
}

esp_err_t max31865_read_temp(max31865_handle_t handle, float *out_temp) {
  if (!handle || !out_temp)
    return ESP_ERR_INVALID_ARG;
  struct max31865_ctx_t *ctx = handle;

  // Trigger one-shot conversion (if we weren't in auto mode, but we set BIAS |
  // ???) Actually, in init we set BIAS on but not Auto. So we need to set
  // 1-Shot bit. Let's read current config, add 1-shot bit, write back.

  uint8_t cfg;
  read_reg(ctx->spi, MAX31865_CONFIG_REG, &cfg);

  // Set 1-Shot bit and ensure BIAS is on
  cfg |= MAX31865_CONFIG_1SHOT | MAX31865_CONFIG_BIAS;
  if (ctx->cfg.three_wire)
    cfg |= MAX31865_CONFIG_3WIRE; // Ensure 3-wire is kept

  write_reg(ctx->spi, MAX31865_CONFIG_REG, cfg);

  // Wait for conversion (approx 65ms for 50Hz filter)
  vTaskDelay(pdMS_TO_TICKS(70));

  // Read RTD Registers (0x01 MSB, 0x02 LSB)
  uint8_t data[2];
  read_regs(ctx->spi, MAX31865_RTD_MSB_REG, data, 2);

  uint16_t rtd_raw = (data[0] << 8) | data[1];

  // Check Fault (D0)
  if (rtd_raw & 1) {
    // Fault detected
    uint8_t fault;
    read_reg(ctx->spi, MAX31865_FAULT_STATUS_REG, &fault);
    ESP_LOGE(
        TAG,
        "MAX31865 Fault detected: 0x%02X (Raw RTD: 0x%04X, Config: 0x%02X)",
        fault, rtd_raw, cfg);

    // Clear fault
    read_reg(ctx->spi, MAX31865_CONFIG_REG, &cfg);
    cfg |= MAX31865_CONFIG_FAULTSTAT; // Write 1 to clear
    write_reg(ctx->spi, MAX31865_CONFIG_REG, cfg);

    *out_temp = 0.0f;
    return ESP_FAIL;
  }

  // Calculate Resistance
  // ADC Code is top 15 bits
  uint16_t adc_code = rtd_raw >> 1;
  float r_rtd = (float)adc_code * ctx->cfg.r_ref / 32768.0f;

  ESP_LOGW(TAG,
           "DEBUG: Raw=0x%04X, ADC=%u, R_ref=%.1f, R0=%.1f, R_RTD=%.2f Ohms",
           rtd_raw, adc_code, ctx->cfg.r_ref, ctx->cfg.r0, r_rtd);

  // Calculate Temperature
  // Callendar-Van Dusen Equation
  // R = R0 * (1 + A*t + B*t^2)
  // 0 = B*t^2 + A*t + (1 - R/R0)

  float Z1 = -RTD_A;
  float Z2 = RTD_A * RTD_A - (4 * RTD_B);
  float Z3 = (4 * RTD_B) / ctx->cfg.r0;
  float Z4 = 2 * RTD_B;

  float temp = Z2 + (Z3 * r_rtd);

  // SOLVING: 0 = B*t^2 + A*t + (1 - R/R0)
  // t = (-A + sqrt(A^2 - 4B(1-R/R0))) / 2B
  // Discriminant D = Z2 + Z3*R
  // Root = (-A + sqrt(D)) / 2B
  // Z1 = -A, Z4 = 2B

  // PREVIOUSLY: (Z1 - sqrt(temp)) / Z4 -> Gave ~6700 C (Incorrect Root)
  // CORRECT: (Z1 + sqrt(temp)) / Z4

  temp = (Z1 + sqrtf(temp)) / Z4;

  // Simplified using user's PT500:
  // t = (-A + sqrt(A*A - 4*B*(1 - R/R0))) / (2*B)

  *out_temp = temp;

  // ESP_LOGD(TAG, "ADC: %d, R: %.2f, T: %.2f", adc_code, r_rtd, temp);
  return ESP_OK;
}

esp_err_t max31865_del(max31865_handle_t handle) {
  if (!handle)
    return ESP_ERR_INVALID_ARG;
  struct max31865_ctx_t *ctx = handle;

  spi_bus_remove_device(ctx->spi);
  // Note: We do not free the bus here as it might be used by other devices
  // or we don't track if we initialized it.

  free(ctx);
  return ESP_OK;
}
