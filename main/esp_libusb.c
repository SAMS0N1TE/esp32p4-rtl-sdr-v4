#include "usb/usb_host.h"
#include "esp_log.h"
#include "esp_libusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static class_adsb_dev *adsbdev;
static usb_transfer_t *s_bulk_transfer = NULL;

void init_adsb_dev()
{
    if (adsbdev) return;
    
    adsbdev = calloc(1, sizeof(class_adsb_dev));
    adsbdev->is_adsb = true;
    adsbdev->response_buf = calloc(256, sizeof(uint8_t));
    adsbdev->done_sem = xSemaphoreCreateBinary();  // ADD THIS
    
    esp_err_t r = usb_host_transfer_alloc(256, 0, &adsbdev->transfer);
    if (r != ESP_OK) {
        ESP_LOGE(TAG_ADSB, "Failed to allocate control transfer");
    }
}

void bulk_transfer_read_cb(usb_transfer_t *transfer)
{
    adsbdev->is_success = (transfer->status == 0);
    adsbdev->bytes_transferred = transfer->actual_num_bytes;
    xSemaphoreGive(adsbdev->done_sem);
}

void transfer_read_cb(usb_transfer_t *transfer)
{
    for (int i = 0; i < transfer->actual_num_bytes; i++) {
        adsbdev->response_buf[i] = transfer->data_buffer[i];
    }
    adsbdev->is_success = (transfer->status == 0);
    adsbdev->bytes_transferred = transfer->actual_num_bytes - sizeof(usb_setup_packet_t);
    xSemaphoreGive(adsbdev->done_sem);
}

int esp_libusb_bulk_transfer(class_driver_t *driver_obj, unsigned char endpoint, unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    // Allocate the Bulk Transfer ONCE and reuse it
    if (!s_bulk_transfer) {
        size_t sizePacket = usb_round_up_to_mps(length, 512);
        esp_err_t r = usb_host_transfer_alloc(sizePacket, 0, &s_bulk_transfer);
        if (r != ESP_OK) {
            ESP_LOGE(TAG_ADSB, "Failed to allocate bulk transfer");
            vTaskDelay(pdMS_TO_TICKS(100)); // Prevent tight loop watchdog crash
            return -1;
        }
    }

    s_bulk_transfer->num_bytes = usb_round_up_to_mps(length, 512);
    s_bulk_transfer->device_handle = driver_obj->dev_hdl;
    s_bulk_transfer->timeout_ms = timeout;
    s_bulk_transfer->bEndpointAddress = endpoint;
    s_bulk_transfer->callback = bulk_transfer_read_cb;
    s_bulk_transfer->context = (void *)&driver_obj;
    
    esp_err_t r = usb_host_transfer_submit(s_bulk_transfer);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG_ADSB, "esp_libusb_bulk_transfer failed to submit: %d", r);
        vTaskDelay(pdMS_TO_TICKS(100)); // Prevent CPU watchdog crash
        return -1;
    }
    
    // DELETE the entire while (!adsbdev->is_done) loop, replace with:
	if (xSemaphoreTake(adsbdev->done_sem, pdMS_TO_TICKS(timeout + 500)) != pdTRUE) {
		ESP_LOGE(TAG_ADSB, "Bulk transfer timed out");
		return -1;
	}
    
    if (!adsbdev->is_success)
    {
        ESP_LOGW(TAG_ADSB, "esp_libusb_bulk_transfer STALL/Fail");
        usb_host_endpoint_clear(driver_obj->dev_hdl, endpoint); // Safely clear bulk halt
        vTaskDelay(pdMS_TO_TICKS(100)); // Prevent CPU watchdog crash
        return -1;
    }
    
    *transferred = adsbdev->bytes_transferred;
    memcpy(data, s_bulk_transfer->data_buffer, *transferred);
    return 0;
}

int esp_libusb_control_transfer(class_driver_t *driver_obj, uint8_t bm_req_type, uint8_t b_request, uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength, unsigned int timeout)
{
    if (!adsbdev->transfer) return -1;

    size_t sizePacket = sizeof(usb_setup_packet_t) + wLength;
    
    USB_SETUP_PACKET_INIT_CONTROL((usb_setup_packet_t *)adsbdev->transfer->data_buffer, bm_req_type, b_request, wValue, wIndex, wLength);
    
    adsbdev->transfer->num_bytes = sizePacket;
    adsbdev->transfer->device_handle = driver_obj->dev_hdl;
    adsbdev->transfer->timeout_ms = timeout;
    adsbdev->transfer->context = (void *)&driver_obj;
    adsbdev->transfer->callback = transfer_read_cb;

    if (bm_req_type == CTRL_OUT && data && wLength > 0)
    {
        for (uint8_t i = 0; i < wLength; i++)
        {
            adsbdev->transfer->data_buffer[sizeof(usb_setup_packet_t) + i] = data[i];
        }
    }
    
    esp_err_t r = usb_host_transfer_submit_control(driver_obj->client_hdl, adsbdev->transfer);
    if (r != ESP_OK)
    {
        ESP_LOGE(TAG_ADSB, "libusb_control_transfer failed to submit: %d", r);
        vTaskDelay(pdMS_TO_TICKS(50)); // CRITICAL: Give ESP-IDF Hub task time to recover EP0
        return -1;
    }

	if (xSemaphoreTake(adsbdev->done_sem, pdMS_TO_TICKS(timeout + 500)) != pdTRUE) {
		ESP_LOGE(TAG_ADSB, "Control transfer timed out");
		return -1;
	}
    
    if (!adsbdev->is_success)
    {
        ESP_LOGW(TAG_ADSB, "libusb_control_transfer STALL/Fail");
        vTaskDelay(pdMS_TO_TICKS(50)); // CRITICAL: Give ESP-IDF Hub task time to clear EP0 STALL
        return -1;
    }
    
    if (bm_req_type == CTRL_IN && data && wLength > 0)
    {
        for (uint8_t i = 0; i < wLength; i++)
        {
            data[i] = adsbdev->response_buf[sizeof(usb_setup_packet_t) + i];
        }
    }
    
    return adsbdev->bytes_transferred;
}

void esp_libusb_get_string_descriptor_ascii(const usb_str_desc_t *str_desc, char *str)
{
    if (str_desc == NULL)
    {
        return;
    }

    for (int i = 0; i < str_desc->bLength / 2; i++)
    {
        str[i] = (char)str_desc->wData[i];
    }
}