#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "drivers/led_driver.h"

#define LED_PIN GPIO_NUM_17
static const char *TAG = "LED_TEST";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LED test...");

    ESP_LOGI(TAG, "Initializing LED driver...");
    led_init(LED_PIN);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Turning LED on...");
    led_set(true);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Turning LED off...");
    led_set(false);
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        ESP_LOGI(TAG, "Toggling LED state...");
        led_toggle();
        ESP_LOGI(TAG, "LED state: %s", led_get_state() ? "ON" : "OFF");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}