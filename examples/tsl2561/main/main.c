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

#include "tsl2561.h"


#define TSL2561_ISR_ENABLE


#define TSL2561_TASK_NAME           "tsl2561-task"
#define TSL2561_TASK_STACK_SIZE     4096
#define TSL2561_TASK_PRIORITY       8

#define TSL2561_MSG_MAX             10


static QueueHandle_t  tsl2561_msg_queue = NULL;
static TaskHandle_t   tsl2561_task_id = NULL;
static tsl2561_t      tsl2561_hid = NULL;

typedef enum {
  MSG_TYPE_ISR,
  MSG_TYPE_POLLING,

} msg_type_e;


typedef struct {
  msg_type_e  type;
  union {
    tsl2561_t handle;
  } u;

} msg_t;

static const char* TAG = "ESP::MAIN::TSL2561";



static void tsl2561_Interrupt(tsl2561_t handle) {
  msg_t msg = { 
    .type = MSG_TYPE_ISR,
    .u.handle = handle
  };
  xQueueSendFromISR(tsl2561_msg_queue, &msg, NULL);
}

static esp_err_t tsl2561_SendMsg(const msg_t* msg) {
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);
  if (xQueueSend(tsl2561_msg_queue, msg, (TickType_t) 0) != pdPASS) {
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
static void tsl2561_TaskFn(void* param) {
  msg_t msg = { 
    .type = MSG_TYPE_POLLING,
  };
  bool    power = false;
  uint8_t id = 0;
  uint16_t broadband = 0;
  uint16_t infrared = 0;
  uint32_t lux = 0;
  float    ratio = 0.0;
  
  bool    loop = true;
  esp_err_t result = ESP_OK;

  ESP_LOGI(TAG, "++%s()", __func__);

  ESP_ERROR_CHECK(tsl2561_Init(&tsl2561_hid));
  //ESP_ERROR_CHECK(tsl2561_SetPower(tsl2561_hid, true));
  ESP_ERROR_CHECK(tsl2561_GetPower(tsl2561_hid, &power));
  ESP_ERROR_CHECK(tsl2561_GetId(tsl2561_hid, &id));
  ESP_LOGD(TAG, "[%s] Power: %d", __func__, power);
  ESP_LOGD(TAG, "[%s]    Id: 0x%02X", __func__, id);
  
  ESP_ERROR_CHECK(result = tsl2561_GetLight(tsl2561_hid, &broadband, &infrared, &lux, &ratio));
  ESP_LOGD(TAG, "[%s] BR: %d, IR: %d, LUX: %ld, ratio: %f", __func__, broadband, infrared, lux, ratio);

#ifdef TSL2561_ISR_ENABLE
  uint16_t lux_threshold = 2000;
  tsl2561_threshold_t threshold = {
    .min = 2000,
    .max = 10000
  };

  msg.type = MSG_TYPE_ISR;

  if (broadband < lux_threshold) {
    threshold.min = 0;
    threshold.max = lux_threshold;
  } else {
    threshold.min = lux_threshold;
    threshold.max = 0xFFFF;
  }

  ESP_ERROR_CHECK(tsl2561_ClearIsr(tsl2561_hid));

  ESP_ERROR_CHECK(tsl2561_SetIsrConfig(tsl2561_hid, true));
  ESP_ERROR_CHECK(tsl2561_SetIsrNotify(tsl2561_hid, tsl2561_Interrupt));

  ESP_ERROR_CHECK(tsl2561_SetIsr(tsl2561_hid, false));
  ESP_LOGV(TAG, "[%s] THRESHOLD ==> min: %d, max: %d", __func__, threshold.min, threshold.max);
  ESP_ERROR_CHECK(tsl2561_SetIsrThreshold(tsl2561_hid, &threshold));
  ESP_ERROR_CHECK(tsl2561_SetIsr(tsl2561_hid, true));
  //ESP_ERROR_CHECK(tsl2561_ClearIsr(tsl2561_hid));

#else
  msg.type = MSG_TYPE_POLLING;
  tsl2561_SendMsg(&msg);
#endif

  while (loop) {
    ESP_LOGD(TAG, "[%s] Wait...", __func__);
    if(xQueueReceive(tsl2561_msg_queue, &msg, portMAX_DELAY) == pdTRUE) {

      ESP_LOGD(TAG, "[%s] Message arrived: type: %d", __func__, msg.type);
      switch (msg.type) {
        case MSG_TYPE_ISR: {
          ESP_LOGD(TAG, "[%s] MSG_TYPE_ISR", __func__);
          ESP_ERROR_CHECK(tsl2561_ClearIsr(tsl2561_hid));

          //result = tsl2561_GetLux(tsl2561_hid, &lux);
          result = tsl2561_GetLight(tsl2561_hid, &broadband, &infrared, &lux, &ratio);
          if (result == ESP_OK) {
            ESP_LOGV(TAG, "[%s] INTERRUPT ==> BR: %d, IR: %d, LUX: %ld, ratio: %f", __func__, broadband, infrared, lux, ratio);

            if (broadband < lux_threshold) {
              threshold.min = 0;
              threshold.max = lux_threshold;
            } else {
              threshold.min = lux_threshold;
              threshold.max = 0xFFFF;
            }

            ESP_ERROR_CHECK(tsl2561_SetIsr(tsl2561_hid, false));
            ESP_LOGV(TAG, "[%s] THRESHOLD ==> min: %d, max: %d", __func__, threshold.min, threshold.max);
            ESP_ERROR_CHECK(tsl2561_SetIsrThreshold(tsl2561_hid, &threshold));
            ESP_ERROR_CHECK(tsl2561_SetIsr(tsl2561_hid, true));

            //ESP_ERROR_CHECK(tsl2561_ClearIsr(tsl2561_hid));
          }
          break;
        }
        case MSG_TYPE_POLLING: {
          ESP_LOGD(TAG, "[%s] MSG_TYPE_POLLING", __func__);
          //result = tsl2561_GetLux(tsl2561_hid, &lux);
          result = tsl2561_GetLight(tsl2561_hid, &broadband, &infrared, &lux, &ratio);
          if (result == ESP_OK) {
            ESP_LOGV(TAG, "[%s] POLLING ===> BR: %d, IR: %d, LUX: %ld, ratio: %f", __func__, broadband, infrared, lux, ratio);
          }
          vTaskDelay(pdMS_TO_TICKS(200));
          
          tsl2561_SendMsg(&msg);

          break;
        }
        default: {
          ESP_LOGW(TAG, "[%s] Unknown Message type: %d", __func__, msg.type);
          break;
        }
      }
    }
  }

  ESP_ERROR_CHECK(tsl2561_SetPower(tsl2561_hid, false));
  ESP_ERROR_CHECK(tsl2561_Done(tsl2561_hid));

  ESP_LOGI(TAG, "--%s()", __func__);
}


void app_main(void)
{
  esp_err_t result = ESP_OK;

  esp_log_level_set(TAG, ESP_LOG_VERBOSE);

  ESP_LOGI(TAG, "++%s()", __func__);

  tsl2561_msg_queue = xQueueCreate(TSL2561_MSG_MAX, sizeof(msg_t));
  if (tsl2561_msg_queue == NULL)
  {
    ESP_LOGE(TAG, "[%s] xQueueCreate() failed.", __func__);
    return;
  }

  xTaskCreate(tsl2561_TaskFn, TSL2561_TASK_NAME, TSL2561_TASK_STACK_SIZE, NULL, TSL2561_TASK_PRIORITY, &tsl2561_task_id);
  if (tsl2561_task_id == NULL)
  {
    ESP_LOGE(TAG, "[%s] xTaskCreate() failed.", __func__);
    return;
  }
  ESP_LOGI(TAG, "--%s() - result: %d", __func__, result);
  return;
}
