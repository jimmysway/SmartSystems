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
#include "driver/i2c.h"
#include <string.h>

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

float speed_cnt = 0;

volatile int wall = 0;

//////////////////////////////////////////////LIDAR//////////////////////////////////////////////
// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// LIDARLITE
#define LIDARLITE_ADDR_DEFAULT 0x62

static void i2c_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;   
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                        I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                        I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    if (err == ESP_OK) {printf("- initialized: yes\n");}

    // Data in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
        printf( "- Device found at address: 0x%X%s", i, "\n");
        count++;
        }
    }
    if (count == 0) {printf("- No I2C devices found!" "\n");}
}


// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
    //adapted from esp idf i2c_simple_main program
    uint8_t write_buf[2] = {reg, data};
    int ret;
    ret = i2c_master_write_to_device(I2C_EXAMPLE_MASTER_NUM, LIDARLITE_ADDR_DEFAULT, write_buf, sizeof(write_buf), 1000/ portTICK_PERIOD_MS);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
    //adapted from esp idf i2c_simple_main program
    uint8_t data;
    i2c_master_write_read_device(I2C_EXAMPLE_MASTER_NUM, LIDARLITE_ADDR_DEFAULT, &reg, 1, &data, 1, 1000 / portTICK_PERIOD_MS);
    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    // YOUR CODE HERE
    //adapted from esp idf i2c_simple_main program

    int16_t data, MSB, LSB;
    
    LSB = readRegister(reg);
    MSB = readRegister(reg + 1);
    data = MSB;
    data = LSB | (MSB << 8);
    return data;
}
//   configuration:  Default 0.
//     0: Maximum range. Uses maximum acquisition count.
//     1: Balanced performance.
//     2: Short range, high speed. Reduces maximum acquisition count.
//     3: Mid range, higher speed. Turns on quick termination
//          detection for faster measurements at short range (with decreased
//          accuracy)
//     4: Maximum range, higher speed on short range targets. Turns on quick
//          termination detection for faster measurements at short range (with
//          decreased accuracy)1
//     5: Very short range, higher speed, high error. Reduces maximum
//          acquisition count to a minimum for faster rep rates on very
//          close targets with high error.
void configure(uint8_t configuration, uint8_t lidarliteAddress)
{
    uint8_t sigCountMax = 0;
    uint8_t acqConfigReg = 0;

    switch (configuration)
    {
        case 0: // Default mode - Maximum range
            sigCountMax     = 0xff;
            acqConfigReg    = 0x08;
            break;

        case 1: // Balanced performance
            sigCountMax     = 0x80;
            acqConfigReg    = 0x08;
            break;

        case 2: // Short range, high speed
            sigCountMax     = 0x18;
            acqConfigReg    = 0x00;
            break;

        case 3: // Mid range, higher speed on short range targets
            sigCountMax     = 0x80;
            acqConfigReg    = 0x00;
            break;

        case 4: // Maximum range, higher speed on short range targets
            sigCountMax     = 0xff;
            acqConfigReg    = 0x00;
            break;

        case 5: // Very short range, higher speed, high error
            sigCountMax     = 0x04;
            acqConfigReg    = 0x00;
            break;
    }

    writeRegister(0x05, sigCountMax);
    writeRegister(0xE5, acqConfigReg);
}

uint8_t getBusyFlag(uint8_t lidarliteAddress)
{
    uint8_t  statusByte = 0;
    uint8_t  busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    statusByte = readRegister(0x01);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;
    return busyFlag;
}

void waitForBusy(uint8_t lidarliteAddress)
{
    uint8_t  busyFlag;

    do
    {
        busyFlag = getBusyFlag(lidarliteAddress);
    } while (busyFlag);

}

void takeRange(uint8_t lidarliteAddress)
{
    uint8_t dataByte = 0x04;
    writeRegister(0x00, dataByte);
}


uint16_t readDistance(uint8_t lidarliteAddress)
{
    uint16_t distance;
    // Read two bytes from registers 0x10 and 0x11
    distance = read16(0x10);
    // printf("Reading %d: %d\n",i, distance);
    return distance;
}

/////////////////////////////////////////////Wheel Speed/////////////////////////////////////////////

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
    
    uint8_t *data = (uint8_t *) malloc(1024);
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
        // speed_cnt = 0;
        // ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
        // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        // vTaskDelay(pdMS_TO_TICKS(2000));
        if (wall == 1) {
            speed_cnt = 0.0;
            ESP_LOGI(TAG, "Speed of buggy STOPPED: %f", speed_cnt);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            speed_cnt = 0.15;
            ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
    //     speed_cnt = 0.14;
    //     ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
    //     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
    //     vTaskDelay(pdMS_TO_TICKS(1000));
        
    //     speed_cnt = 0.15;
    //     ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
    //     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
    //     vTaskDelay(pdMS_TO_TICKS(1000));
        
    //     speed_cnt = 0.16;
    //     ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
    //     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
    //     vTaskDelay(pdMS_TO_TICKS(2000));
        
    //     speed_cnt = 0;
    //     ESP_LOGI(TAG, "Speed of buggy: %f", speed_cnt);
    //     ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
    //     vTaskDelay(pdMS_TO_TICKS(100));
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
        angle = -22;
        // ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle)));
        // if(angle == -22) {
        //     vTaskDelay(pdMS_TO_TICKS(1975));
        // }
        //Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(25));
        // if ((angle + step) > 10 || (angle + step) < -60) {
        //     step *= -1;
        // }
        // angle += step;
    }
}

void lidar_task() 
{
    printf("\n>> Polling LIDAR\n");
    int numTrials = 5;
    while (1) {
        int cumDistance = 0;
        for (int i = 0; i < numTrials; i++) {
            waitForBusy(LIDARLITE_ADDR_DEFAULT);
            takeRange(LIDARLITE_ADDR_DEFAULT);
            cumDistance += readDistance(LIDARLITE_ADDR_DEFAULT);
        }
        if(cumDistance / numTrials < 75)
        {
            speed_cnt = 0.0;
            ESP_LOGI(TAG, "WALL DETECTED");
            wall = 1;
            //ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, speed_to_compare(speed_cnt)));
        } else {
            wall = 0;
        }
        printf("Distance: %d\n", cumDistance / numTrials);
        vTaskDelay(30/portTICK_PERIOD_MS);
    }
}

void app_main(void) {

    i2c_master_init();
    i2c_scanner();
    configure(2, LIDARLITE_ADDR_DEFAULT);
    // Create the servo task
    xTaskCreate(lidar_task, "LidarTask", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(servo_task, "ServoTask", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(calibrateESC, "calibrateESC", 4096, NULL, configMAX_PRIORITIES - 2, NULL);

    xTaskCreate(encoder_task, "EncoderTask", 4096, NULL, configMAX_PRIORITIES-3, NULL);   

    xTaskCreate(timer, "timer", 4096, NULL, configMAX_PRIORITIES-4, NULL);
}