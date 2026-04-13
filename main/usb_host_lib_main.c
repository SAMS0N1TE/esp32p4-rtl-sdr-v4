/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "driver/gpio.h"

#define HOST_LIB_TASK_PRIORITY 2
#define CLASS_TASK_PRIORITY 3
#define APP_QUIT_PIN CONFIG_APP_QUIT_PIN

#define USB_VBUS_EN_GPIO 46 // P4 VBUS Power Switch Pin

#ifdef CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
#define ENABLE_ENUM_FILTER_CALLBACK
#endif // CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK

extern void class_driver_task(void *arg);
extern void class_driver_client_deregister(void);

static const char *TAG = "USB host lib";

QueueHandle_t app_event_queue = NULL;

typedef enum
{
    APP_EVENT = 0,
} app_event_group_t;

typedef struct
{
    app_event_group_t event_group;
} app_event_queue_t;

static void gpio_cb(void *arg)
{
    const app_event_queue_t evt_queue = {
        .event_group = APP_EVENT,
    };

    BaseType_t xTaskWoken = pdFALSE;

    if (app_event_queue)
    {
        xQueueSendFromISR(app_event_queue, &evt_queue, &xTaskWoken);
    }

    if (xTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

#ifdef ENABLE_ENUM_FILTER_CALLBACK
static bool set_config_cb(const usb_device_desc_t *dev_desc, uint8_t *bConfigurationValue)
{
    if (dev_desc->bNumConfigurations > 1)
    {
        *bConfigurationValue = 2;
    }
    else
    {
        *bConfigurationValue = 1;
    }

    return true;
}
#endif // ENABLE_ENUM_FILTER_CALLBACK

static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
#ifdef ENABLE_ENUM_FILTER_CALLBACK
        .enum_filter_cb = set_config_cb,
#endif // ENABLE_ENUM_FILTER_CALLBACK
        .peripheral_map = BIT0,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "USB Host installed with peripheral map 0x%x", host_config.peripheral_map);

    xTaskNotifyGive(arg);

    bool has_clients = true;
    bool has_devices = false;
    while (has_clients)
    {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            ESP_LOGI(TAG, "Get FLAGS_NO_CLIENTS");
            if (ESP_OK == usb_host_device_free_all())
            {
                ESP_LOGI(TAG, "All devices marked as free, no need to wait FLAGS_ALL_FREE event");
                has_clients = false;
            }
            else
            {
                ESP_LOGI(TAG, "Wait for the FLAGS_ALL_FREE");
                has_devices = true;
            }
        }
        if (has_devices && event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            ESP_LOGI(TAG, "Get FLAGS_ALL_FREE");
            has_clients = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices, uninstall USB Host library");

    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskSuspend(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB host library example");

    // ENABLE USB VBUS POWER FOR RTL-SDR
    ESP_LOGI(TAG, "Enabling USB VBUS on GPIO %d", USB_VBUS_EN_GPIO);
    gpio_config_t vbus_cfg = {
        .pin_bit_mask = (1ULL << USB_VBUS_EN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&vbus_cfg));
    ESP_ERROR_CHECK(gpio_set_level(USB_VBUS_EN_GPIO, 1));
    
    // Give the RTL-SDR half a second to charge its capacitors before enumeration
    vTaskDelay(pdMS_TO_TICKS(500)); 

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_isr_handler_add(APP_QUIT_PIN, gpio_cb, NULL));

    app_event_queue = xQueueCreate(10, sizeof(app_event_queue_t));
    app_event_queue_t evt_queue;

    TaskHandle_t host_lib_task_hdl, class_driver_task_hdl;

    BaseType_t task_created;
    task_created = xTaskCreatePinnedToCore(usb_host_lib_task,
                                           "usb_host",
                                           4096,
                                           xTaskGetCurrentTaskHandle(),
                                           HOST_LIB_TASK_PRIORITY,
                                           &host_lib_task_hdl,
                                           0);
    assert(task_created == pdTRUE);

    ulTaskNotifyTake(false, 1000);

    task_created = xTaskCreatePinnedToCore(class_driver_task,
                                           "class",
                                           5 * 1024,
                                           NULL,
                                           CLASS_TASK_PRIORITY,
                                           &class_driver_task_hdl,
                                           0);
    assert(task_created == pdTRUE);
    vTaskDelay(10);

    while (1)
    {
        if (xQueueReceive(app_event_queue, &evt_queue, portMAX_DELAY))
        {
            if (APP_EVENT == evt_queue.event_group)
            {
                usb_host_lib_info_t lib_info;
                ESP_ERROR_CHECK(usb_host_lib_info(&lib_info));
                if (lib_info.num_devices != 0)
                {
                    ESP_LOGW(TAG, "Shutdown with attached devices.");
                }
                break;
            }
        }
    }

    class_driver_client_deregister();
    vTaskDelay(10);

    vTaskDelete(class_driver_task_hdl);
    vTaskDelete(host_lib_task_hdl);

    gpio_isr_handler_remove(APP_QUIT_PIN);
    xQueueReset(app_event_queue);
    vQueueDelete(app_event_queue);
    ESP_LOGI(TAG, "End of the example");
}