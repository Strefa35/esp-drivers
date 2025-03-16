/**
 * @file tsl2561.h
 * @author A.Czerwinski@pistacje.net
 * @brief 
 * @version 0.1
 * @date 2025-03-06
 * 
 * @copyright Copyright (c) 2025 4Embedded.Systems
 * 
 */

#ifndef __TSL2561_H__
#define __TSL2561_H__


#include <stdio.h>
#include <stdbool.h>

#include "esp_err.h"

struct tsl2561_s;
typedef struct tsl2561_s* tsl2561_t;


esp_err_t tsl2561_setPower(const tsl2561_t handle, const bool on);
esp_err_t tsl2561_getPower(const tsl2561_t handle, bool* on);
esp_err_t tsl2561_getId(const tsl2561_t handle, uint8_t *id);

esp_err_t tsl2561_Init(tsl2561_t* const handle);
esp_err_t tsl2561_Done(tsl2561_t handle);


#endif /* __TSL2561_H__ */