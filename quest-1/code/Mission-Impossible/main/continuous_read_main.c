
#include <stdio.h>
#include <string.h>
#include "esp_vfs_dev.h"	// This is associated with VFS -- virtual file system interface and abstraction -- see the docs
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define Red 26
#define Green 25
#define Yellow 4
#define Blue 5

#define Mode 18
#define Button 34

static const char *TAG = "RTOS Task";


static void init(void) // Iniialize pins and directions
{
    ESP_LOGI(TAG, "Configured GPIO LED!");
    gpio_reset_pin(Red);
    gpio_reset_pin(Green);
    gpio_reset_pin(Yellow);
    gpio_reset_pin(Blue);

    gpio_reset_pin(Button);


    /* Set the GPIO as a push/pull output */
    gpio_set_direction(Red, GPIO_MODE_OUTPUT);
    gpio_set_direction(Green, GPIO_MODE_OUTPUT);
    gpio_set_direction(Yellow, GPIO_MODE_OUTPUT);
    gpio_set_direction(Blue, GPIO_MODE_OUTPUT);

    gpio_set_direction(Button, GPIO_MODE_INPUT);

}

static void led_function()
{
    int led = 1;
    while(gpio_get_level(Button) == 0 || gpio_get_level(Button) == 1) // Polling loop
    {
        if(gpio_get_level(Button) == 1) //  Check if button is pressed
        {
            led++; // Increment led variable
            if(led == 5)
            {
                led = 1;
            }
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        if(led == 1) // Set LEDs based on led variable
        {
            gpio_set_level(Red, 1);
            gpio_set_level(Green, 0);
            gpio_set_level(Yellow, 0);
            gpio_set_level(Blue, 0);
        }
        else if(led == 2)
        {
            gpio_set_level(Red, 0);
            gpio_set_level(Green, 1);
            gpio_set_level(Yellow, 0);
            gpio_set_level(Blue, 0);
        }
        else if(led == 3)
        {
            gpio_set_level(Red, 0);
            gpio_set_level(Green, 0);
            gpio_set_level(Yellow, 1);
            gpio_set_level(Blue, 0);
        }
        else 
        {
            gpio_set_level(Red, 0);
            gpio_set_level(Green, 0);
            gpio_set_level(Yellow, 0);
            gpio_set_level(Blue, 1);
        }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}



void app_main()
{
    init();

    led_function();

}