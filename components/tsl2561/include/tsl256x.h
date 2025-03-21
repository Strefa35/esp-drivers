/**
 * @file tsl256x.h
 * @author A.Czerwinski@pistacje.net
 * @brief Light to digital converter for TSL2560 & TSL2561
 * @version 0.1
 * @date 2025-03-16
 * 
 * @copyright Copyright (c) 2025 4Embedded.Systems
 * 
 */

#ifndef __TSL256X_H__
#define __TSL256X_H__


#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"


typedef enum {
  GAIN_1X   = 0x00,
  GAIN_10X  = 0x10
} tsl256x_gain_e;

typedef enum {
  INTEG_13MS  = 0,
  INTEG_101MS,
  INTEG_402MS,
  INTEG_NA
} tsl256x_integ_e;

typedef struct {
  uint32_t  min;
  uint32_t  max;
} tsl256x_threshold_t;

struct tsl256x_s;
typedef struct tsl256x_s* tsl256x_t;

typedef void(*tsl256x_isr_f)(tsl256x_t handle);

esp_err_t tsl256x_setPower(const tsl256x_t handle, const bool on);
esp_err_t tsl256x_getPower(const tsl256x_t handle, bool* on);
esp_err_t tsl256x_getId(const tsl256x_t handle, uint8_t *id);
esp_err_t tsl256x_setGain(const tsl256x_t handle, const tsl256x_gain_e gain);
esp_err_t tsl256x_setIntegrationTime(const tsl256x_t handle, const tsl256x_integ_e time);
esp_err_t tsl256x_getLux(const tsl256x_t handle, uint32_t* lux);

esp_err_t tsl256x_setThreshold(const tsl256x_t handle, const tsl256x_threshold_t* threshold);

esp_err_t tsl256x_RegisterIsr(tsl256x_t handle, tsl256x_isr_f fn);
esp_err_t tsl256x_UnregisterIsr(tsl256x_t handle);
esp_err_t tsl256x_ClearIsr(const tsl256x_t handle);

esp_err_t tsl256x_Init(tsl256x_t* const handle);
esp_err_t tsl256x_Done(tsl256x_t handle);


#endif /* __TSL256X_H__ */