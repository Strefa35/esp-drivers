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

#include "tsl2561.h"


#define TSL2561_TASK_NAME           "tsl2561-task"
#define TSL2561_TASK_STACK_SIZE     4096
#define TSL2561_TASK_PRIORITY       8


static TaskHandle_t tsl2561_task_id = NULL;
static tsl2561_t    tsl2561_hid = NULL;

static const char* TAG = "ESP::MAIN::TSL2561";


/**
 * @brief Task's function
 * 
 * @param param 
 */
static void tsl2561_TaskFn(void* param) {
  bool    power = false;
  uint8_t id = 0;
  bool    loop = true;
  //esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);

  ESP_ERROR_CHECK(tsl2561_Init(&tsl2561_hid));
  ESP_ERROR_CHECK(tsl2561_setPower(tsl2561_hid, true));
  ESP_ERROR_CHECK(tsl2561_getPower(tsl2561_hid, &power));
  ESP_ERROR_CHECK(tsl2561_getId(tsl2561_hid, &id));

  ESP_LOGD(TAG, "[%s] Power: %d", __func__, power);
  ESP_LOGD(TAG, "[%s]    Id: 0x%02X", __func__, id);
  
  while (loop) {
    ESP_LOGD(TAG, "[%s] Wait...", __func__);

    vTaskDelay(pdMS_TO_TICKS(1000));



  }

  ESP_ERROR_CHECK(tsl2561_setPower(tsl2561_hid, false));
  ESP_ERROR_CHECK(tsl2561_Done(tsl2561_hid));

  ESP_LOGI(TAG, "--%s()", __func__);
}


void app_main(void)
{
  esp_err_t result = ESP_OK;

  esp_log_level_set(TAG, CONFIG_TSL2561_LOG_LEVEL);

  ESP_LOGI(TAG, "++%s()", __func__);

  xTaskCreate(tsl2561_TaskFn, TSL2561_TASK_NAME, TSL2561_TASK_STACK_SIZE, NULL, TSL2561_TASK_PRIORITY, &tsl2561_task_id);
  if (tsl2561_task_id == NULL)
  {
    ESP_LOGE(TAG, "[%s] xTaskCreate() failed.", __func__);
    return;
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return;
}
