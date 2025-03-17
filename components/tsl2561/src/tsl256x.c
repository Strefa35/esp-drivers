/**
 * @file tsl256x.c
 * @author A.Czerwinski@pistacje.net
 * @brief Light to digital converter for TSL2560 & TSL2561
 * @version 0.1
 * @date 2025-03-16
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

#include "tsl256x.h"
#include "tsl256x_lux.h"


// TSL256X
#define TSL256X_PORT_NUMBER           0
#define TSL256X_SLAVE_ADDR            0x39


/* ======================== */
/*     Register Address     */
/* ======================== */

#define TSL256X_REG_COMMAND           0x80

/* 0x00 - Control of basic functions  */
#define	TSL256X_REG_CONTROL           0x00

/* 0x01 - Integration time/gain control */
#define	TSL256X_REG_TIMING            0x01

/* 0x02 - Low byte of low interrupt threshold   */ 
/* 0x03 - High byte of low interrupt threshold  */
#define	TSL256X_REG_THRESH_L          0x02

/* 0x04 - Low byte of high interrupt threshold  */
/* 0x05 - High byte of high interrupt threshold */
#define	TSL256X_REG_THRESH_H          0x04

/* Interrupt */
#define	TSL256X_REG_INTCTL            0x06

/* Part number/ Rev ID - */
#define	TSL256X_REG_ID                0x0A

/* 0x0C - Low byte of ADC channel 0   */
/* 0x0D - High byte of ADC channel 0  */
#define	TSL256X_REG_DATA_0            0x0C

/* 0x0E - Low byte of ADC channel 1   */
/* 0x0F - High byte of ADC channel 1  */
#define	TSL256X_REG_DATA_1            0x0E

/* ======================== */
/*     Command Register     */
/* ======================== */
#define TSL256X_CMD_CLEAR             0xC0
#define TSL256X_CMD_WORD_PROTOCOL     0x20
#define TSL256X_CMD_BLOCK_PROTOCOL    0x10

/* ======================== */
/*     Control Register     */
/* ======================== */
#define TSL256X_CTRL_POWER_ON         0x03
#define TSL256X_CTRL_POWER_OFF        0x00

/* ======================== */
/*     Timing Register      */
/* ======================== */
#define TSL256X_TIMING_GAIN           0x10
#define TSL256X_TIMING_MANUAL         0x80
#define TSL256X_TIMING_INTEGRATE      0x03


/* GPIO Configuration */
#define TSL256X_SDA_GPIO              13
#define TSL256X_SCL_GPIO              16

//#define TSL256X_I2C_CLK_FREQUENCY    400000
#define TSL256X_I2C_CLK_FREQUENCY     100000

typedef enum {
  PART_TSL2560CS        = 0x00,
  PART_TSL2561CS        = 0x10,
  PART_TSL2560T_FN_CL   = 0x40,
  PART_TSL2561T_FN_CL   = 0x50,

  PART_TSL256X_UNKNOWN  = 0xFF
} tsl256x_partno_e;

typedef struct tsl256x_s {
  i2c_master_bus_handle_t bus;
  i2c_master_dev_handle_t device;
  tsl256x_gain_e          gain;
  tsl256x_integ_e         integ;
  tsl256x_partno_e        partno;
  uint8_t                 revno;
} tsl256x_s;


static const i2c_master_bus_config_t tsl256x_bus_config = {
  .clk_source = I2C_CLK_SRC_DEFAULT,
  .i2c_port = TSL256X_PORT_NUMBER,
  .scl_io_num = TSL256X_SCL_GPIO,
  .sda_io_num = TSL256X_SDA_GPIO,
  .glitch_ignore_cnt = 7,
  .flags.enable_internal_pullup = true,
};

static const i2c_device_config_t tsl256x_dev_cfg = {
  .dev_addr_length = I2C_ADDR_BIT_LEN_7,
  .device_address = TSL256X_SLAVE_ADDR,
  .scl_speed_hz = TSL256X_I2C_CLK_FREQUENCY,
};


static const char* TAG = "ESP::DRV::TLS256X";

static void tsl256x_print(const tsl256x_t handle) {
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
static esp_err_t tsl256x_init(tsl256x_t* const handle_ptr) {
  tsl256x_t handle = NULL;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle_ptr: %p)", __func__, handle_ptr);

  if (handle_ptr == NULL) {
    ESP_LOGE(TAG, "[%s] handle_ptr=NULL", __func__);
    return ESP_FAIL;
  }

  handle = malloc(sizeof(tsl256x_s));
  if (handle == NULL) {
    ESP_LOGE(TAG, "[%s] Memory allocation problem", __func__);
    result = ESP_ERR_NO_MEM;
  }

  memset(handle, 0x00, sizeof(tsl256x_s));

  ESP_LOGD(TAG, "Initialize I2C bus");
  result = i2c_new_master_bus(&tsl256x_bus_config, &(handle->bus));
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] i2c_new_master_bus() failed: %d.", __func__, result);
    free(handle);
    return result;
  }

  ESP_ERROR_CHECK(i2c_master_probe(handle->bus, TSL256X_SLAVE_ADDR, -1));

  ESP_LOGD(TAG, "Add device to the bus");
  result = i2c_master_bus_add_device(handle->bus, &tsl256x_dev_cfg, &(handle->device));

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
static esp_err_t tsl256x_done(tsl256x_t handle) {
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
static esp_err_t tsl256x_read(const tsl256x_t handle, uint8_t* data_ptr, size_t data_size) {
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
static esp_err_t tsl256x_write(const tsl256x_t handle, const uint8_t* data_ptr, const size_t data_size) {
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
static esp_err_t tsl256x_readId(const tsl256x_t handle) {
  uint8_t cmd = { TSL256X_REG_COMMAND | TSL256X_REG_ID };
  uint8_t data;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl256x_read(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_read() - failed: %d.", __func__, result);
    return result;
  }
  handle->partno  = data & 0xF0;
  handle->revno   = data & 0x0F;
  ESP_LOGD(TAG, "--> REG: 0x%02X", data);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Read Timing Register (GAIN & INTEG)
 *
 * @param handle
 * @return esp_err_t
 */
static esp_err_t tsl256x_readTiming(const tsl256x_t handle) {
  uint8_t cmd = { TSL256X_REG_COMMAND | TSL256X_REG_TIMING };
  uint8_t data;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl256x_read(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_read() - failed: %d.", __func__, result);
    return result;
  }
  handle->gain  = data & TSL256X_TIMING_GAIN;
  handle->integ = data & TSL256X_TIMING_INTEGRATE;
  ESP_LOGD(TAG, "--> REG: 0x%02X", data);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Write Gain
 *
 * @param handle
 * @param gain
 * @return esp_err_t
 */
static esp_err_t tsl256x_writeGain(const tsl256x_t handle, const tsl256x_gain_e gain) {
  uint8_t cmd = { TSL256X_REG_COMMAND | TSL256X_REG_TIMING };
  uint8_t data = handle->integ | gain;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, gain: 0x%02X)", __func__, handle, gain);
  result = tsl256x_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl256x_write(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(data: 0x%02X) - failed: %d.", __func__, data, result);
    return result;
  }
  handle->gain  = gain;
  ESP_LOGD(TAG, "--> REG: 0x%02X", data);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Write Integration Time
 *
 * @param handle
 * @param time
 * @return esp_err_t
 */
static esp_err_t tsl256x_WriteIntegrationTime(const tsl256x_t handle, const tsl256x_integ_e time) {
  uint8_t cmd = { TSL256X_REG_COMMAND | TSL256X_REG_TIMING };
  uint8_t data = handle->integ | time;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, time: 0x%02X)", __func__, handle, time);
  result = tsl256x_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl256x_write(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(data: 0x%02X) - failed: %d.", __func__, data, result);
    return result;
  }
  handle->integ  = time;
  ESP_LOGD(TAG, "--> REG: 0x%02X", data);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Read Data from Channels
 *
 * @param handle
 * @return esp_err_t
 */
static esp_err_t tsl256x_readData(const tsl256x_t handle, uint16_t* data0, uint16_t* data1) {
  uint8_t cmd_ch0 = { TSL256X_REG_COMMAND | TSL256X_CMD_WORD_PROTOCOL | TSL256X_REG_DATA_0 };
  uint8_t cmd_ch1 = { TSL256X_REG_COMMAND | TSL256X_CMD_WORD_PROTOCOL | TSL256X_REG_DATA_1 };
  uint8_t data_ch0[2];
  uint8_t data_ch1[2];
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);

  /* Read data from channel 0 */
  result = tsl256x_write(handle, &cmd_ch0, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd_ch0: 0x%02X) - failed: %d.", __func__, cmd_ch0, result);
    return result;
  }
  result = tsl256x_read(handle, data_ch0, 2);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_read() - failed: %d.", __func__, result);
    return result;
  }
  ESP_LOGD(TAG, "--> DATA[0]: 0x%02X 0x%02X", data_ch0[0], data_ch0[1]);
  *data0 = (data_ch0[1] << 8) | data_ch0[0];

  /* Read data from channel 1 */
  result = tsl256x_write(handle, &cmd_ch1, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd_ch1: 0x%02X) - failed: %d.", __func__, cmd_ch1, result);
    return result;
  }
  result = tsl256x_read(handle, data_ch1, 2);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_read() - failed: %d.", __func__, result);
    return result;
  }
  ESP_LOGD(TAG, "--> DATA[1]: 0x%02X 0x%02X", data_ch1[0], data_ch1[1]);
  *data1 = (data_ch1[1] << 8) | data_ch1[0];

  ESP_LOGD(TAG, "--> DATA -> ch0: %d, ch1: %d", *data0, *data1);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

static esp_err_t tsl256x_getFactor(const tsl256x_t handle, const uint32_t ratio, uint32_t* b, uint32_t* m) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, ratio: %ld)", __func__, handle, ratio);
  switch (handle->partno) {
    case PART_TSL2560CS:
    case PART_TSL2561CS: {

      if (ratio <= K1C) {
        *b = B1C;
        *m = M1C;
      } else if (ratio <= K2C) {
        *b = B2C;
        *m = M2C;
      } else if (ratio <= K3C) {
        *b = B3C;
        *m = M3C;
      } else if (ratio <= K4C) {
        *b = B4C;
        *m = M4C;
      } else if (ratio <= K5C) {
        *b = B5C;
        *m = M5C;
      } else if (ratio <= K6C) {
        *b = B6C;
        *m = M6C;
      } else if (ratio <= K7C) {
        *b = B7C;
        *m = M7C;
      } else if (ratio > K8C) {
        *b = B8C;
        *m = M8C;
      }       
      break;
    }
    case PART_TSL2560T_FN_CL:
    case PART_TSL2561T_FN_CL: {
      if (ratio <= K1T) {
        *b = B1T; 
        *m = M1T;
      } else if (ratio <= K2T) {
        *b = B2T;
        *m = M2T;
      } else if (ratio <= K3T) {
        *b = B3T;
        *m = M3T;
      } else if (ratio <= K4T) {
        *b = B4T;
        *m = M4T;
      } else if (ratio <= K5T) {
        *b = B5T;
        *m = M5T;
      } else if (ratio <= K6T) {
        *b = B6T;
        *m = M6T;
      } else if (ratio <= K7T) {
        *b = B7T;
        *m = M7T;
      } else if (ratio > K8T) {
        *b = B8T;
        *m = M8T;
      }
      break;
    }
    default: {
      ESP_LOGE(TAG, "[%s] Invalid Part Number Identification: 0x%02X ", __func__, handle->partno);
      return ESP_ERR_NOT_SUPPORTED;
    }
  }
  ESP_LOGD(TAG, "--> Factors: B: %ld, M: %ld", *b, *m);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Calculating Lux
 * 
 * @param handle 
 * @param data0 
 * @param data1 
 * @param lux 
 * @return esp_err_t 
 */
static esp_err_t tsl256x_calculateLux(const tsl256x_t handle, const uint16_t ch0, const uint16_t ch1, uint32_t* lux) {
  uint32_t chScale;
  uint32_t channel0;
  uint32_t channel1;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, ch0: %d, ch1: %d)", __func__, handle, ch0, ch1);
  switch (handle->integ) {
    case INTEG_13MS: {
      chScale = CHSCALE_TINT0;
      break;
    }
    case INTEG_101MS: {
      chScale = CHSCALE_TINT1;
      break;
    }
    default: {
      chScale = (1 << CH_SCALE);
      break;
    }
  }
  ESP_LOGD(TAG, "--> chScale: 0x%08lX", chScale);

  // scale if gain is NOT 16X
  if (handle->gain == GAIN_1X) {
    chScale = chScale << 4; // scale 1X to 16X  
  }

  // scale the channel values
  channel0 = (ch0 * chScale) >> CH_SCALE;
  channel1 = (ch1 * chScale) >> CH_SCALE;  

  // find the ratio of the channel values (Channel1/Channel0)
  // protect against divide by zero
  uint32_t ratio1 = 0;
  if (channel0 != 0) {
    ratio1 = (channel1 << (RATIO_SCALE + 1)) / channel0;
  }
  // round the ratio value
  uint32_t ratio = (ratio1 + 1) >> 1;
  uint32_t b;
  uint32_t m;

  result = tsl256x_getFactor(handle, ratio, &b, &m);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_getFactor() - failed: %d", __func__, result);
    return result;
  }

  uint32_t temp = ((channel0 * b) - (channel1 * m));

  // round lsb (2^(LUX_SCALE−1))
  temp += (1 << (LUX_SCALE - 1));

  // strip off fractional portion
  *lux = temp >> LUX_SCALE;

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
esp_err_t tsl256x_setPower(const tsl256x_t handle, const bool on) {
  uint8_t cmd[] = { 
    TSL256X_REG_COMMAND | TSL256X_REG_CONTROL, 
    on == true ? TSL256X_CTRL_POWER_ON : TSL256X_CTRL_POWER_OFF
  };
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_write(handle, cmd, 2);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Get Power state (On/Off)
 * 
 * @param handle 
 * @return esp_err_t 
 */
esp_err_t tsl256x_getPower(const tsl256x_t handle, bool* on) {
  uint8_t cmd = { TSL256X_REG_COMMAND | TSL256X_REG_CONTROL };
  uint8_t data;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_write(handle, &cmd, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_write(cmd: 0x%02X) - failed: %d.", __func__, cmd, result);
    return result;
  }
  result = tsl256x_read(handle, &data, 1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_read() - failed: %d.", __func__, result);
    return result;
  }
  *on = data & 0x03 ? true : false;
  ESP_LOGD(TAG, "--> POWER: 0x%02X", data);
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
esp_err_t tsl256x_getId(const tsl256x_t handle, uint8_t *id) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_readId(handle);
  if (result == ESP_OK) {
    *id = handle->partno | handle->revno;
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Set Gain
 * 
 * @param handle 
 * @param gain 
 * @return esp_err_t 
 */
esp_err_t tsl256x_setGain(const tsl256x_t handle, const tsl256x_gain_e gain) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, gain: 0x%02X)", __func__, handle, gain);
  result = tsl256x_writeGain(handle, gain);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Set Integration Time
 * 
 * @param handle 
 * @param time 
 * @return esp_err_t 
 */
esp_err_t tsl256x_setIntegrationTime(const tsl256x_t handle, const tsl256x_integ_e time) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p, time: %d)", __func__, handle, time);
  result = tsl256x_WriteIntegrationTime(handle, time);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Get current lux
 * 
 * @param handle 
 * @param lux 
 * @return esp_err_t 
 */
esp_err_t tsl256x_getLux(const tsl256x_t handle, uint32_t* lux) {
  uint16_t data0 = 0;
  uint16_t data1 = 0;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_readData(handle, &data0, &data1);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "[%s] tsl256x_readData() - result: %d.", __func__, result);
    return result;
  }
  result = tsl256x_calculateLux(handle, data0, data1, lux);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

/**
 * @brief Initialize TSL256X driver
 * 
 * @return esp_err_t 
 */
esp_err_t tsl256x_Init(tsl256x_t* const handle_ptr) {
  esp_err_t result = ESP_OK;

  esp_log_level_set(TAG, CONFIG_TSL256X_LOG_LEVEL);

  ESP_LOGI(TAG, "++%s(handle_ptr: %p)", __func__, handle_ptr);
  if (handle_ptr != NULL) {
    result = tsl256x_init(handle_ptr);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl256x_init() - result: %d.", __func__, result);
      return result;
    }

    result = tsl256x_readId(*handle_ptr);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl256x_readId() - result: %d.", __func__, result);
      return result;
    }

    result = tsl256x_readTiming(*handle_ptr);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl256x_readTiming() - result: %d.", __func__, result);
      return result;
    }

    tsl256x_print(*handle_ptr);

  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}

esp_err_t tsl256x_Done(tsl256x_t handle) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s(handle: %p)", __func__, handle);
  result = tsl256x_done(handle);
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}
