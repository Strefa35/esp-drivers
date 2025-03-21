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
#include "freertos/queue.h"

#include "esp_err.h"

#include "tsl256x.h"


#define TSL256X_ISR_ENABLE


#define TSL256X_TASK_NAME           "tsl256x-task"
#define TSL256X_TASK_STACK_SIZE     4096
#define TSL256X_TASK_PRIORITY       8

#define TSL256X_MSG_MAX             10


static QueueHandle_t  tsl256x_msg_queue = NULL;
static TaskHandle_t   tsl256x_task_id = NULL;
static tsl256x_t      tsl256x_hid = NULL;

typedef enum {
  MSG_TYPE_ISR,
  MSG_TYPE_POLLING,

} msg_type_e;


typedef struct {
  msg_type_e  type;
  union {
    tsl256x_t handle;
  } u;

} msg_t;

static const char* TAG = "ESP::MAIN::TSL256X";



static void tsl256x_Interrupt(tsl256x_t handle) {
  msg_t msg = { 
    .type = MSG_TYPE_ISR,
    .u.handle = handle
  };
  xQueueSendFromISR(tsl256x_msg_queue, &msg, NULL);
}

static esp_err_t tsl256x_SendMsg(const msg_t* msg) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);
  if (xQueueSend(tsl256x_msg_queue, msg, (TickType_t) 0) != pdPASS) {
    ESP_LOGE(TAG, "[%s] Message error. type: %d", __func__, msg->type);
    result = ESP_FAIL;
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return result;
}


/**
 * @brief Task's function
 * 
 * @param param 
 */
static void tsl256x_TaskFn(void* param) {
  msg_t msg = { 
    .type = MSG_TYPE_POLLING,
  };
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

#ifdef TSL256X_ISR_ENABLE
  tsl256x_threshold_t threshold = {
    .min = 50,
    .max = 10000
  };

  msg.type = MSG_TYPE_ISR;

  ESP_ERROR_CHECK(tsl256x_RegisterIsr(tsl256x_hid, tsl256x_Interrupt));
  ESP_ERROR_CHECK(tsl256x_setThreshold(tsl256x_hid, &threshold));

#else
  msg.type = MSG_TYPE_POLLING;
  tsl256x_SendMsg(&msg);
#endif

  while (loop) {
    ESP_LOGD(TAG, "[%s] Wait...", __func__);
    if(xQueueReceive(tsl256x_msg_queue, &msg, portMAX_DELAY) == pdTRUE) {
      uint32_t lux = 0;

      ESP_LOGD(TAG, "[%s] Message arrived: type: %d", __func__, msg.type);
      switch (msg.type) {
        case MSG_TYPE_ISR: {
          ESP_LOGD(TAG, "[%s] MSG_TYPE_ISR", __func__);
          tsl256x_ClearIsr(tsl256x_hid);
          result = tsl256x_getLux(tsl256x_hid, &lux);
          if (result == ESP_OK) {
            ESP_LOGI(TAG, "[%s] LUX: %ld", __func__, lux);
          }
          break;
        }
        case MSG_TYPE_POLLING: {
          ESP_LOGD(TAG, "[%s] MSG_TYPE_POLLING", __func__);
          result = tsl256x_getLux(tsl256x_hid, &lux);
          if (result == ESP_OK) {
            ESP_LOGI(TAG, "[%s] LUX: %ld", __func__, lux);
          }
          vTaskDelay(pdMS_TO_TICKS(1000));
          
          tsl256x_SendMsg(&msg);

          break;
        }
        default: {
          ESP_LOGD(TAG, "[%s] Unknown Message type: %d", __func__, msg.type);
          break;
        }
      }
    }
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

  tsl256x_msg_queue = xQueueCreate(TSL256X_MSG_MAX, sizeof(msg_t));
  if (tsl256x_msg_queue == NULL)
  {
    ESP_LOGE(TAG, "[%s] xQueueCreate() failed.", __func__);
    return;
  }

  xTaskCreate(tsl256x_TaskFn, TSL256X_TASK_NAME, TSL256X_TASK_STACK_SIZE, NULL, TSL256X_TASK_PRIORITY, &tsl256x_task_id);
  if (tsl256x_task_id == NULL)
  {
    ESP_LOGE(TAG, "[%s] xTaskCreate() failed.", __func__);
    return;
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return;
}
