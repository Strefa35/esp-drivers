/**
 * @file tsl2561.c
 * @author A.Czerwinski@pistacje.net
 * @brief 
 * @version 0.1
 * @date 2025-03-06
 * 
 * @copyright Copyright (c) 2025 4Embedded.Systems
 * 
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "sdkconfig.h"

#include "driver/gpio.h"

#include "driver/i2c_master.h"

#include "esp_log.h"
#include "esp_err.h"

#include "tsl2561.h"

//TSL2561
#define TSL2561_PORT_NUMBER           0
#define TSL2561_SLAVE_ADDR            0x39


/* ======================== */
/*     Register Address     */
/* ======================== */

#define TSL2561_REG_COMMAND           0x80

/* 0x00 - Control of basic functions  */
#define	TSL2561_REG_CONTROL           0x00

/* 0x01 - Integration time/gain control */
#define	TSL2561_REG_TIMING            0x01

/* 0x02 - Low byte of low interrupt threshold   */ 
/* 0x03 - High byte of low interrupt threshold  */
#define	TSL2561_REG_THRESH_L          0x02

/* 0x04 - Low byte of high interrupt threshold  */
/* 0x05 - High byte of high interrupt threshold */
#define	TSL2561_REG_THRESH_H          0x04

/* Interrupt */
#define	TSL2561_REG_INTCTL            0x06

/* Part number/ Rev ID - */
#define	TSL2561_REG_ID                0x0A

/* 0x0C - Low byte of ADC channel 0   */
/* 0x0D - High byte of ADC channel 0  */
#define	TSL2561_REG_DATA_0            0x0C

/* 0x0E - Low byte of ADC channel 1   */
/* 0x0F - High byte of ADC channel 1  */
#define	TSL2561_REG_DATA_1            0x0E

/* ======================== */
/*     Command Register     */
/* ======================== */
#define TSL2561_CMD_CLEAR             0xC0
#define TSL2561_CMD_WORD_PROTOCOL     0x20
#define TSL2561_CMD_BLOCK_PROTOCOL    0x10

/* ======================== */
/*     Control Register     */
/* ======================== */
#define TSL2561_CTRL_POWER_ON         0x03
#define TSL2561_CTRL_POWER_OFF        0x00

/* ======================== */
/*     Timing Register      */
/* ======================== */
#define TSL2561_TIMING_GAIN           0x10
#define TSL2561_TIMING_MANUAL         0x80
#define TSL2561_TIMING_INTEGRATE      0x03


/* GPIO Configuration */
#define TSL2561_SDA_GPIO             13
#define TSL2561_SCL_GPIO             16

//#define TSL2561_I2C_CLK_FREQUENCY    400000
#define TSL2561_I2C_CLK_FREQUENCY    100000

typedef enum {
  PART_TSL2560CS        = 0x00,
  PART_TSL2561CS        = 0x01,
  PART_TSL2560T_FN_CL   = 0x04,
  PART_TSL2561T_FN_CL   = 0x05,

  PART_TSL256X_UNKNOWN  = 0xFF
} tsl256x_partno_e;

typedef enum {
  GAIN_1X   = 0,
  GAIN_10X  = 1
} tsl256x_gain_e;

typedef enum {
  INTEG_13MS    = 0,
  INTEG_101MS,
  INTEG_402MS,
  INTEG_NA
} tsl256x_integ_e;

typedef struct tsl2561_s {
  i2c_master_bus_handle_t bus;
  i2c_master_dev_handle_t device;
  tsl256x_gain_e          gain;
  tsl256x_integ_e         integ;
  tsl256x_partno_e        partno;
  uint8_t                 revno;

} tsl2561_s;


static const i2c_master_bus_config_t tsl2561_bus_config = {
  .clk_source = I2C_CLK_SRC_DEFAULT,
  .i2c_port = TSL2561_PORT_NUMBER,
  .scl_io_num = TSL2561_SCL_GPIO,
  .sda_io_num = TSL2561_SDA_GPIO,
  .glitch_ignore_cnt = 7,
  .flags.enable_internal_pullup = true,
};

static const i2c_device_config_t tsl2561_dev_cfg = {
  .dev_addr_length = I2C_ADDR_BIT_LEN_7,
  .device_address = TSL2561_SLAVE_ADDR,
  .scl_speed_hz = TSL2561_I2C_CLK_FREQUENCY,
};


static const char* TAG = "ESP::DRV::TLS2561";

static void tsl2561_print(const tsl2561_t handle) {
  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  ESP_LOGD(TAG, "   bus: %p", handle->bus);
  ESP_LOGD(TAG, "device: %p", handle->device);
  ESP_LOGD(TAG, "partno: %d", handle->partno);
  ESP_LOGD(TAG, " revno: %d", handle->revno);
  ESP_LOGD(TAG, "  gain: %d", handle->gain);
  ESP_LOGD(TAG, " integ: %d", handle->integ);
  ESP_LOGI(TAG, "--%s()", __func__);
}

/**
 * @brief Initialize I2C bus and add device to the bus
 * 
 * @return esp_err_t 
 */
static esp_err_t tsl2561_init(tsl2561_t* const handle_ptr) {
  tsl2561_t handle = NULL;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle_ptr: %p)", __func__, handle_ptr);

  if (handle_ptr == NULL) {
    ESP_LOGE(TAG, "[%s] handle_ptr=NULL", __func__);
    return ESP_FAIL;
  }

  handle = malloc(sizeof(tsl2561_s));
  if (handle == NULL) {
    ESP_LOGE(TAG, "[%s] Memory allocation problem", __func__);
    result = ESP_ERR_NO_MEM;
  }

  memset(handle, 0x00, sizeof(tsl2561_s));

  ESP_LOGD(TAG, "Initialize I2C bus");
  result = i2c_new_master_bus(&tsl2561_bus_config, &(handle->bus));
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] i2c_new_master_bus() failed: %d.", __func__, result);
    free(handle);
    return result;
  }

  ESP_ERROR_CHECK(i2c_master_probe(handle->bus, TSL2561_SLAVE_ADDR, -1));

  ESP_LOGD(TAG, "Add device to the bus");
  result = i2c_master_bus_add_device(handle->bus, &tsl2561_dev_cfg, &(handle->device));

  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] i2c_master_bus_add_device() failed: %d.", __func__, result);
    free(handle);
    return result;
  }
  *handle_ptr = handle;
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Remove device from the bus and delete bus
 * 
 * @return esp_err_t 
 */
static esp_err_t tsl2561_done(tsl2561_t handle) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);

  if (handle->device) {
    result = i2c_master_bus_rm_device(handle->device);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] i2c_master_bus_rm_device() failed: %d.", __func__, result);
    }
  }

  if (handle->bus != NULL) {
    result = i2c_del_master_bus(handle->bus);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] i2c_del_master_bus() failed: %d.", __func__, result);
    }
  }

  free(handle);
  handle = NULL;

  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Read data from the I2C device
 * 
 * @param data_ptr - buffer to fill a data
 * @param data_size - size of buffer / data to read
 * @return esp_err_t 
 */
static esp_err_t tsl2561_read(const tsl2561_t handle, uint8_t* data_ptr, size_t data_size) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, data_ptr: %p, data_size: %d)", __func__, handle, data_ptr, data_size);
  result = i2c_master_receive(handle->device, data_ptr, data_size, -1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] i2c_master_transmit() failed: %d.", __func__, result);
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Write command to the I2C device
 * 
 * @param data_ptr - buffer with data to send
 * @param data_size - size of buffer / data to send
 * @return esp_err_t 
 */
static esp_err_t tsl2561_write(const tsl2561_t handle, const uint8_t* data_ptr, const size_t data_size) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, data_ptr: %p, data_size: %d)", __func__, handle, data_ptr, data_size);
  result = i2c_master_transmit(handle->device, data_ptr, data_size, -1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] i2c_master_transmit() failed: %d.", __func__, result);
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Read ID Register (PARTNO & REVNO)
 *
 * @param handle
 * @return esp_err_t
 */
static esp_err_t tsl2561_readId(const tsl2561_t handle) {
  uint8_t cmd = { TSL2561_REG_COMMAND | TSL2561_REG_ID };
  uint8_t data;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl2561_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl2561_read(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_read() - failed: %d.", __func__, result);
    return result;
  }
  ESP_LOGD(TAG, "--> REG: 0x%02X", data);
  handle->partno  = (data & 0xF0) >> 4;
  handle->revno   = data & 0x0F;

  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Read Timing Register (PARTNO & REVNO)
 *
 * @param handle
 * @return esp_err_t
 */
static esp_err_t tsl2561_readTiming(const tsl2561_t handle) {
  uint8_t cmd = { TSL2561_REG_COMMAND | TSL2561_REG_TIMING };
  uint8_t data;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl2561_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl2561_read(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_read() - failed: %d.", __func__, result);
    return result;
  }
  ESP_LOGD(TAG, "--> REG: 0x%02X", data);
  handle->gain  = (data & 0x1) >> 4;
  handle->integ = data & 0x03;
  
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}


/* ================================================================================== */

/**
 * @brief Set Power On/Off
 * 
 * @param handle 
 * @param on 
 * @return esp_err_t 
 */
esp_err_t tsl2561_setPower(const tsl2561_t handle, const bool on) {
  uint8_t cmd[] = { 
    TSL2561_REG_COMMAND | TSL2561_REG_CONTROL, 
    on == true ? TSL2561_CTRL_POWER_ON : TSL2561_CTRL_POWER_OFF
  };
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);
  result = tsl2561_write(handle, cmd, 2);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Get Power state (On/Off)
 * 
 * @param handle 
 * @return esp_err_t 
 */
esp_err_t tsl2561_getPower(const tsl2561_t handle, bool* on) {
  uint8_t cmd = { TSL2561_REG_COMMAND | TSL2561_REG_CONTROL };
  uint8_t data;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);
  result = tsl2561_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl2561_read(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_read() - failed: %d.", __func__, result);
    return result;
  }
  ESP_LOGD(TAG, "--> POWER: 0x%02X", data);
  *on = data & 0x03 ? true : false;
  
  ESP_LOGI(TAG, "--%s(on: %d) - result: %d", __func__, *on, result);
  return result;
}

/**
 * @brief Get ID (PARTNO & REVNO)
 * 
 * @param handle 
 * @param data 
 * @return esp_err_t 
 */
esp_err_t tsl2561_getId(const tsl2561_t handle, uint8_t *id) {
  uint8_t cmd = { TSL2561_REG_COMMAND | TSL2561_REG_ID };
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);
  result = tsl2561_readId(handle);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl2561_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  *id = handle->partno;
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Initialize TSL2561 driver
 * 
 * @return esp_err_t 
 */
esp_err_t tsl2561_Init(tsl2561_t* const handle_ptr) {
  esp_err_t result = ESP_OK;

  esp_log_level_set(TAG, CONFIG_TSL2561_LOG_LEVEL);

  ESP_LOGI(TAG, "++%s(handle_ptr: %p)", __func__, handle_ptr);
  if (handle_ptr != NULL) {
    result = tsl2561_init(handle_ptr);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl2561_init() - result: %d.", __func__, result);
      return result;
    }
    result = tsl2561_readId(*handle_ptr);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl2561_readId() - result: %d.", __func__, result);
      return result;
    }

    result = tsl2561_readTiming(*handle_ptr);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl2561_readTiming() - result: %d.", __func__, result);
      return result;
    }

    tsl2561_print(*handle_ptr);

  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

esp_err_t tsl2561_Done(tsl2561_t handle) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl2561_done(handle);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}
