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


static const bool TSL2561_ISR_ENABLE = false;


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

/**
 * @brief Interrupt notify function
 * 
 * @param handle 
 */
static void tsl2561_Interrupt(tsl2561_t handle) {
  msg_t msg = { 
    .type = MSG_TYPE_ISR,
    .u.handle = handle
  };
  xQueueSendFromISR(tsl2561_msg_queue, &msg, NULL);
}

/**
 * Send message to the thread
 */
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

  ESP_LOGD(TAG, "[%s] Init TSL2561", __func__);

  ESP_ERROR_CHECK(tsl2561_Init(&tsl2561_hid));
  //ESP_ERROR_CHECK(tsl2561_SetPower(tsl2561_hid, true));
  ESP_ERROR_CHECK(tsl2561_GetPower(tsl2561_hid, &power));
  ESP_ERROR_CHECK(tsl2561_GetId(tsl2561_hid, &id));
  ESP_LOGD(TAG, "[%s] Power: %d", __func__, power);
  ESP_LOGD(TAG, "[%s]    Id: 0x%02X", __func__, id);
  
  ESP_LOGD(TAG, "[%s] Get light from TSL2561", __func__);

  ESP_ERROR_CHECK(result = tsl2561_GetLight(tsl2561_hid, &broadband, &infrared, &lux, &ratio));
  ESP_LOGD(TAG, "[%s] BR: %d, IR: %d, LUX: %ld, ratio: %f", __func__, broadband, infrared, lux, ratio);

  uint16_t lux_threshold = 2000;
  tsl2561_threshold_t threshold = {
    .min = 0,
    .max = 0
  };

  if (TSL2561_ISR_ENABLE == true) {
    msg.type = MSG_TYPE_ISR;

    if (broadband < lux_threshold) {
      threshold.min = 0;
      threshold.max = lux_threshold;
    } else {
      threshold.min = lux_threshold;
      threshold.max = 63000;
    }
  
    ESP_LOGD(TAG, "[%s] Init ISR", __func__);
    ESP_ERROR_CHECK(tsl2561_InitIsr(tsl2561_hid));
  
    ESP_LOGD(TAG, "[%s] Set ISR Notify fn", __func__);
    ESP_ERROR_CHECK(tsl2561_SetIsrNotify(tsl2561_hid, tsl2561_Interrupt));
  
    ESP_LOGV(TAG, "[%s] THRESHOLD ==> min: %d, max: %d", __func__, threshold.min, threshold.max);
    ESP_ERROR_CHECK(tsl2561_SetIsrThreshold(tsl2561_hid, &threshold));
  
    ESP_LOGD(TAG, "[%s] Enable ISR on TSL2561", __func__);
    ESP_ERROR_CHECK(tsl2561_SetIsrControl(tsl2561_hid, CTRL_ISR_ENABLE));
  
    ESP_LOGD(TAG, "[%s] Clear ISR on TSL2561", __func__);
    ESP_ERROR_CHECK(tsl2561_SetIsrControl(tsl2561_hid, CTRL_ISR_CLEAR));
  } else {
    msg.type = MSG_TYPE_POLLING;
    tsl2561_SendMsg(&msg);
  }

  while (loop) {
    ESP_LOGD(TAG, "[%s] Wait...\n\n", __func__);
    if(xQueueReceive(tsl2561_msg_queue, &msg, portMAX_DELAY) == pdTRUE) {

      ESP_LOGD(TAG, "[%s] Message arrived: type: %d", __func__, msg.type);
      switch (msg.type) {

        case MSG_TYPE_ISR: {
          ESP_LOGD(TAG, "[%s] MSG_TYPE_ISR", __func__);

          ESP_LOGD(TAG, "[%s] Disable ISR on TSL2561", __func__);
          ESP_ERROR_CHECK(tsl2561_SetIsrControl(tsl2561_hid, CTRL_ISR_DISABLE));
        
          //result = tsl2561_GetLux(tsl2561_hid, &lux);
          result = tsl2561_GetLight(tsl2561_hid, &broadband, &infrared, &lux, &ratio);
          if (result == ESP_OK) {
            ESP_LOGV(TAG, "[%s] INTERRUPT ==> BR: %d, IR: %d, LUX: %ld, ratio: %f", __func__, broadband, infrared, lux, ratio);

            if (broadband < lux_threshold) {
              threshold.min = 0;
              threshold.max = lux_threshold;
            } else {
              threshold.min = lux_threshold;
              threshold.max = 63000;
            }

            ESP_LOGV(TAG, "[%s] THRESHOLD ==> min: %d, max: %d", __func__, threshold.min, threshold.max);
            ESP_ERROR_CHECK(tsl2561_SetIsrThreshold(tsl2561_hid, &threshold));
          }
          ESP_LOGD(TAG, "[%s] Enable ISR on TSL2561", __func__);
          ESP_ERROR_CHECK(tsl2561_SetIsrControl(tsl2561_hid, CTRL_ISR_ENABLE));
        
          ESP_LOGD(TAG, "[%s] Clear ISR on TSL2561", __func__);
          ESP_ERROR_CHECK(tsl2561_SetIsrControl(tsl2561_hid, CTRL_ISR_CLEAR));
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
  ESP_ERROR_CHECK(tsl2561_DoneIsr(tsl2561_hid));
  ESP_ERROR_CHECK(tsl2561_Done(tsl2561_hid));

  ESP_LOGI(TAG, "--%s()", __func__);
}

/**
 * @brief Main function
 * 
 */
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
