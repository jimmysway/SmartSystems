/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/mcpwm_timer.h"

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"

#include "esp_attr.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle

#define SERVO_PULSE_GPIO             13        // GPIO connects to the PWM signal line
#define ESC_PULSE_GPIO             15        // GPIO connects to the PWM signal line

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define ESC_MIN 900
#define ESC_MAX 2100
#define ESC_NEUTRAL 1500

// For ADC
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

// For ADC
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1 (A2)
static const adc_atten_t atten = ADC_ATTEN_DB_0;
static const adc_unit_t unit = ADC_UNIT_1;

bool pulsed = false;

int count;

// For ADC
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

static void timer() {
    while(1) {
        count = 0;
        vTaskDelay(100);
        printf("Count: %d\n", count);
        float speed = (count * (2*3.14159*7/6))/100; //0.005; //(count / 6) * 0.62;
        printf("Speed is %.1fm/s\n", speed);
    }
}


static inline uint32_t example_angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static inline uint32_t speed_to_compare(float speed) {
    uint32_t calc_speed;
    if(speed >= 0) {
        calc_speed = ((ESC_MAX - ESC_NEUTRAL) * speed) + ESC_NEUTRAL;
    } else {
        calc_speed = ((ESC_NEUTRAL - ESC_MIN) * speed) + ESC_NEUTRAL;
    }
    
    return calc_speed;
}

void calibrateESC(void *param) {
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer2 = NULL;
    mcpwm_timer_config_t timer_config2 = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config2, &timer2));

    mcpwm_oper_handle_t oper2 = NULL;
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer2));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator2 = NULL;
    mcpwm_comparator_config_t comparator_config2 = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config2, &comparator2));

    mcpwm_gen_handle_t generator2 = NULL;
    mcpwm_generator_config_t generator_config2 = {
        .gen_gpio_num = ESC_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer2));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer2, MCPWM_TIMER_START_NO_STOP));
    
//    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));
    
    vTaskDelay(3500 / portTICK_PERIOD_MS); // Do for at least 3s, and leave in neutral state
    
    float speed_cnt = 0;
    char speed_buf[20] = "0";
    float step = 0.02;
//    while (1) {
//        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
//        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
//        vTaskDelay(pdMS_TO_TICKS(500));
//        printf("Set speed of buggy: ");
//        gets(speed_buf);
//        printf("%s\n", speed_buf);
//        speed_cnt = atof(speed_buf);
//    }
    while (1) {
        speed_cnt = 0;
        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        speed_cnt = 0.13;
        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        speed_cnt = 0.14;
        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        speed_cnt = 0.15;
        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        speed_cnt = 0.16;
        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        speed_cnt = 0;
        ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void encoder_task(void *arg)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Initialize counter
    count = 0;

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;

        if (adc_reading < 4095 && pulsed == false)
        {
            count++;
            pulsed = true;
        } else if (adc_reading == 4095) {
            pulsed = false;
        }

        // printf("Raw: %d\n", adc_reading);
        // printf("Count: %d\n", count);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void servo_task(void *param) {
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    int angle = 0;
    int step = 2;
    while (1) {
        // ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
        if(angle == -22) {
            vTaskDelay(pdMS_TO_TICKS(1975));
        }
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(25));
        if ((angle + step) > 10 || (angle + step) < -60) {
            step *= -1;
        }
        angle += step;
    }
}

void app_main(void) {
    // Create the servo task
    xTaskCreate(servo_task,       // Task function
                "ServoTask",      // Name of the task
                4096,             // Stack size (in words)
                NULL,             // Task input parameter
                configMAX_PRIORITIES,                // Priority of the task
                NULL);            // Task handle
    xTaskCreate(calibrateESC,       // Task function
                "calibrateESC",      // Name of the task
                4096,             // Stack size (in words)
                NULL,             // Task input parameter
                configMAX_PRIORITIES - 1,                // Priority of the task
                NULL);            // Task handle

    xTaskCreate(encoder_task,       // Task function
                "EncoderTask",      // Name of the task
                4096,             // Stack size (in words)
                NULL,             // Task input parameter
                configMAX_PRIORITIES-2,                // Priority of the task
                NULL);   

    xTaskCreate(timer, "timer", 4096, NULL, configMAX_PRIORITIES-3, NULL);
}