#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc_cal.h>
#include <driver/gpio.h>
#include <driver/adc.h>

#define PHOTOCELL_PIN 35    
#define THERMISTOR_PIN 34   
#define PUSHBUTTON_PIN 2    

#define GREEN_LED_PIN 3
#define RED_LED_PIN 4
#define YELLOW_LED_PIN 5
#define BLUE_LED_PIN 6

esp_adc_cal_characteristics_t *adc_chars;

void init_adc() {
    //ADC for thermistor
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
    printf("ADC characterization: %d\n", val_type);
}

uint32_t read_adc(uint32_t channel) {
    uint32_t adc_reading = 0;
    for (int i = 0; i < 10; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= 10;
    return esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
}

void init_gpio() {
    // configure GPIO pins for LEDs
    gpio_set_direction(GREEN_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(YELLOW_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLUE_LED_PIN, GPIO_MODE_OUTPUT);
    
    // configure GPIO pin for pushbutton
    gpio_set_direction(PUSHBUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PUSHBUTTON_PIN, GPIO_PULLUP_ONLY);
}

void app_main() {
    init_adc();
    init_gpio();
    
    while (1) {

        uint32_t light_level = read_adc(PHOTOCELL_PIN);
        uint32_t temperature = read_adc(THERMISTOR_PIN);
        int floor_contact = gpio_get_level(PUSHBUTTON_PIN);

        // Display sensor values on the serial console
        printf("Light: %ld\n", light_level);
        printf("Temperature: %ld\n", temperature);
        printf("Floor Contact: %d\n", floor_contact);

        // Check sensor values and activate LEDs accordingly
        if (light_level < 500) {
            gpio_set_level(YELLOW_LED_PIN, 1);  // (yellow LED)
        } else {
            gpio_set_level(YELLOW_LED_PIN, 0);
        }

        if (temperature > 600) {
            gpio_set_level(RED_LED_PIN, 1);  // (red LED)
        } else {
            gpio_set_level(RED_LED_PIN, 0);
        }

        if (floor_contact == 0) {
            gpio_set_level(BLUE_LED_PIN, 1);  // (blue LED)
        } else {
            gpio_set_level(BLUE_LED_PIN, 0);
        }

        // If no faults are detected, turn on green LED
        if (light_level >= 500 && temperature <= 600 && floor_contact == 1) {
            gpio_set_level(GREEN_LED_PIN, 1);  // clear (green LED)
        } else {
            gpio_set_level(GREEN_LED_PIN, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // delay 
    }
}
