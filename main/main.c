/**
 * Copyright (C) 2025 4Embeddes.Systems
 * All rights reserved. 
 * 
 * Find details copyright statement at "LICENSE" file.
 */

#include <stdio.h>
#include <stdbool.h>

#include "sdkconfig.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "tsl256x.h"


#define TSL256X_TASK_NAME           "tsl256x-task"
#define TSL256X_TASK_STACK_SIZE     4096
#define TSL256X_TASK_PRIORITY       8


static TaskHandle_t tsl256x_task_id = NULL;
static tsl256x_t    tsl256x_hid = NULL;

static const char* TAG = "ESP::MAIN::TSL256X";


/**
 * @brief Task's function
 * 
 * @param param 
 */
static void tsl256x_TaskFn(void* param) {
  bool    power = false;
  uint8_t id = 0;
  bool    loop = true;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);

  ESP_ERROR_CHECK(tsl256x_Init(&tsl256x_hid));
  ESP_ERROR_CHECK(tsl256x_setPower(tsl256x_hid, true));
  ESP_ERROR_CHECK(tsl256x_getPower(tsl256x_hid, &power));
  ESP_ERROR_CHECK(tsl256x_getId(tsl256x_hid, &id));

  ESP_LOGD(TAG, "[%s] Power: %d", __func__, power);
  ESP_LOGD(TAG, "[%s]    Id: 0x%02X", __func__, id);
  
  while (loop) {
    uint32_t lux = 0;

    ESP_LOGD(TAG, "[%s] Wait...", __func__);
    result = tsl256x_getLux(tsl256x_hid, &lux);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "[%s] tsl256x_getLux() - result: %d.", __func__, result);
    }
    ESP_LOGI(TAG, "[%s] LUX: %ld", __func__, lux);

    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ESP_ERROR_CHECK(tsl256x_setPower(tsl256x_hid, false));
  ESP_ERROR_CHECK(tsl256x_Done(tsl256x_hid));

  ESP_LOGI(TAG, "--%s()", __func__);
}


void app_main(void)
{
  esp_err_t result = ESP_OK;

  esp_log_level_set(TAG, ESP_LOG_VERBOSE);

  ESP_LOGI(TAG, "++%s()", __func__);
  xTaskCreate(tsl256x_TaskFn, TSL256X_TASK_NAME, TSL256X_TASK_STACK_SIZE, NULL, TSL256X_TASK_PRIORITY, &tsl256x_task_id);
  if (tsl256x_task_id == NULL)
  {
    ESP_LOGE(TAG, "[%s] xTaskCreate() failed.", __func__);
    return;
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return;
}
