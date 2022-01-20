#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "Stepmotor.h"

const char *TAG = "Motor Control";

/*********** Tasks ***********/
void clock(void *arg)
{
    const char *TAG = "Clock";
    int cnt = 0;
    while (1)
    {
        ESP_LOGI(TAG, "%d s", cnt);
        cnt++;
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/*********** Main ***********/
extern "C" void app_main(void)
{
    // 1.Create a motor instance
    Stepmotor motorX(0, 15, 2); // id(0~6), dirPin, stepPin

    xTaskCreate(clock, "clock", 2048, NULL, 5, NULL);

    int cnt = 1;
    while (1)
    {
        // 2.Update motor status
        motorX.isReady = Stepmotor::status[motorX.id];

        // 3.Ask the motor move a little
        if (motorX.isReady)
        {
            ESP_LOGI(TAG, "motorX start moving, %d steps at %d step/s", cnt * 500, 500);
            motorX.move(cnt * 500, 500); // stepNum, speed(step/s)
            cnt++;
        }

        if (cnt == 4)
        {
            ESP_LOGI(TAG, "motorX stop");
            motorX.stop();
            cnt++;
        }

        vTaskDelay(10 / portTICK_RATE_MS);
    }
}