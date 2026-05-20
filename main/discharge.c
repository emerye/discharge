
/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

const static char *TAG = "";

/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define LOAD_ADC_CHANNEL ADC_CHANNEL_3

#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12

#define GPIO_OUTPUT_IO_18 GPIO_NUM_18
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_18)

#define LED_GPIO 48

// #define LOOP_DELAY_TIME 60000
#define LOOP_DELAY_TIME 1000

/*---------------------------------------------------------------
        ADS1116
---------------------------------------------------------------*/

#define I2C_MASTER_SCL_IO 9         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 8         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

// ADS1115
#define ADS1115_ADDR 0x48   /*!< Address of the ADS1x15 sensor */
#define CONVERSION_REG 0X00 /* Conversion Register*/
#define CONFIG_REG 0x01     /* Configuration Register*/

#define BATTERY_MUX 0x42   /*AN0 High Byte Battery connection with proper gain*/
#define REGULATOR_MUX 0x52 /*AN1 High Byte Regulator connection with proper gain*/
#define LOWCONF_BYTE 0xA3  /* 250 SPS and Comparator Disabled*/
#define READ_BATTERY 0     /* Used to select read_adc_voltage battery or regulator*/
#define READ_REGULATOR 1   /* Used to select read_adc_voltage battery or regulator*/

static gpio_config_t io_conf = {};
static void init_gpios();
static int last_reg_voltage;
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static float regulator_volts; 

float calc_voltage(uint8_t, uint8_t);
static esp_err_t read_adc_voltage(int whichVoltage, i2c_master_dev_handle_t dev_handle, float *batVolts);

/**
 * @brief Read a sequence of bytes.
 */
static esp_err_t ads1x15_register_read(i2c_master_dev_handle_t dev_handle, uint8_t *read_buffer, size_t len)
{
    return i2c_master_receive(dev_handle, read_buffer, len, I2C_MASTER_TIMEOUT_MS);
}

/*
 * @brief Write a byte to address pointer
 */
static esp_err_t ads1x15_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t data[], int16_t numbytes)
{
    return (i2c_master_transmit(dev_handle, data, numbytes, I2C_MASTER_TIMEOUT_MS));
}



// LED task
void led_toggle(void *pvParameters)
{
    int current_level = 0;
    while (1)
    {
        if (current_level == 1)
        {
            gpio_set_level(GPIO_OUTPUT_IO_18, 0);
            current_level = 0;
        }
        else
        {
            gpio_set_level(GPIO_OUTPUT_IO_18, 1);
            current_level = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// LDO task
void control_enable(void *)
{
    while (1)
    {
        gpio_set_level(GPIO_OUTPUT_IO_18, 0);
        // LED
        gpio_set_level(GPIO_NUM_48, 1);

        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_ERROR_CHECK(read_adc_voltage(READ_REGULATOR, dev_handle, &regulator_volts));
        printf("Regulator Voltage in task %.3f\n", regulator_volts);

        gpio_set_level(GPIO_OUTPUT_IO_18, 1);
        gpio_set_level(GPIO_NUM_48, 0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


/*
Read either battery voltage using BATTERY or REGULATOR. whichVoltage = 0 for battery else it will return the regulator voltage.
*/
static esp_err_t read_adc_voltage(int whichVoltage, i2c_master_dev_handle_t dev_handle, float *batVolts)
{
    uint8_t data[6];
    uint8_t buffer[6] = {0};

    buffer[0] = 1; // Select control register and write control bytes
    if (whichVoltage == READ_BATTERY)
    {
        buffer[1] = BATTERY_MUX;
    }
    else
    {
        buffer[1] = REGULATOR_MUX;
    }
    buffer[2] = LOWCONF_BYTE;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buffer, 3, data, 2, I2C_MASTER_TIMEOUT_MS));
    buffer[0] = 0;
    ESP_ERROR_CHECK(ads1x15_register_write_byte(dev_handle, buffer, 1));
    ads1x15_register_read(dev_handle, buffer, 2);
    *batVolts = calc_voltage(buffer[0], buffer[1]);
    return 0;
}


/*
 * @brief i2c master initialization1
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

/*
Calculate actual voltage with 4.096 gain
*/
float calc_voltage(uint8_t lowByte, uint8_t highByte)
{
    return ((lowByte * 256 + highByte) * 0.000125);
}

void app_main(void)
{
    double acc_time = 0.0;
    uint32_t sample_number = 0;
    double_t amp_hours = 0;
    double average_current;
    float batVolts;

    init_gpios();

      // Control enable
    xTaskCreatePinnedToCore(
        control_enable,   /* Task function. */
        "control_enable", /* name of task. */
        2048,             /* Stack size of task */
        NULL,             /* parameter of the task */
        5,                /* priority of the task */
        NULL,             /* Task handle to keep track of created task */
        1                 /* tskNO_AFFINITY*/
    );


    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    read_adc_voltage(READ_BATTERY, dev_handle, &batVolts);
    printf("Battery Voltage %.3f\n", batVolts);

    read_adc_voltage(READ_REGULATOR, dev_handle, &batVolts);
    printf("Regulator Voltage %.3f\n", batVolts);

    usleep(2000);
    while (1)
    {
        read_adc_voltage(READ_BATTERY, dev_handle, &batVolts);
        printf("Battery Voltage %.3f\n", batVolts);

        sleep(2.0);

        read_adc_voltage(READ_REGULATOR, dev_handle, &batVolts);
        printf("Regulator Voltage %.3f\n", batVolts);

        acc_time = acc_time + 0.016666666;
        sample_number = sample_number + 1;
        average_current = ((last_reg_voltage / 1000.0) / 60.0 * 0.5); // 3v / 60 ohms  50% on time 50% off.

        amp_hours = amp_hours + (average_current * 0.016666666666); // 1 min / 60 min
                                                                    
        //  printf("%lu ETime %.3f Hrs BatV %d mV RegV %d mV %.3f aH\n", sample_number, acc_time, voltage[0][0], last_reg_voltage, amp_hours);
    }

    usleep(1000);

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void init_gpios()
{
    // zero-initialize the config structure.
    // gpio_config_t io_conf = {};
    // disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    // disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

#ifdef DEBUG
void gpio_toggle()
{
    int cnt = 0;
    while (1)
    {
        printf("cnt: %d\n", cnt++);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_IO_18, cnt % 2);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_IO_18, cnt % 2);
    }
}
#endif
