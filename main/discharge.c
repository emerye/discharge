
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
#include <time.h>
#include <sys/time.h>
#include "esp_efuse.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/i2c_master.h"
#include "esp_mac.h"

const static char *TAG = "";

// 5 seconds
//#define MAINLOOP_DELAY  5000

// One minute update
// #define MAINLOOP_DELAY  60000

// Five minute update
// Time in minutes
#define UPDATE_RATE_DURATION    0.1

/* 1M and 2M Battery divider correction*/
#define BATTERY_DIVIDER_CORRECTION     1.5f
/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_2
#define LDO_ADC_CHANNEL ADC_CHANNEL_3

#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12

#define GPIO_OUTPUT_IO_18 GPIO_NUM_18
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_18)

#define LED_GPIO 48

// #define LOOP_DELAY_TIME 60000
// #define LOOP_DELAY_TIME 1000

/*---------------------------------------------------------------
        ADS1115
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

// Manually set the time of day
static struct tm tm;
 


static gpio_config_t io_conf = {};
static void init_gpios();
/*  I2C 
static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;
static float regulator_volts = 0;
static float batVolts;
*/
static int batteryVoltage;
static int regulatorVoltage;  

//ESP32-s3 ADC
static int adc_raw[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
static adc_cali_handle_t adc1_cali_chan1_handle = NULL;
static adc_oneshot_unit_handle_t adc1_handle;
static bool do_calibration1_chan0;
static bool do_calibration1_chan1;

/*
float calc_voltage(uint8_t, uint8_t);
static esp_err_t read_adc_voltage(int whichVoltage, i2c_master_dev_handle_t dev_handle, float *batVolts);
*/

/**
 * @brief Read a sequence of bytes.
 
static esp_err_t ads1x15_register_read(i2c_master_dev_handle_t dev_handle, uint8_t *read_buffer, size_t len)
{
    return i2c_master_receive(dev_handle, read_buffer, len, I2C_MASTER_TIMEOUT_MS);
}
*/

/*
 * @brief Write a byte to address pointer
 */
/*
static esp_err_t ads1x15_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t data[], int16_t numbytes)
{
    return (i2c_master_transmit(dev_handle, data, numbytes, I2C_MASTER_TIMEOUT_MS));
}
*/

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
        
        gpio_set_level(GPIO_NUM_48, 1);     //LED
      
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_raw[0][0]));
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &batteryVoltage));
            batteryVoltage = (float)batteryVoltage * BATTERY_DIVIDER_CORRECTION;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(GPIO_OUTPUT_IO_18, 1);
        gpio_set_level(GPIO_NUM_48, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));
        
          ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LDO_ADC_CHANNEL, &adc_raw[0][1]));
        if (do_calibration1_chan1) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &regulatorVoltage));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


/*
Read either battery voltage using BATTERY or REGULATOR. whichVoltage = 0 for battery else it will return the regulator voltage.
*/

/*
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
    ESP_ERROR_CHECK(ads1x15_register_write_byte(dev_handle, buffer, 3));
    buffer[0] = 0;
    ESP_ERROR_CHECK(ads1x15_register_write_byte(dev_handle, buffer, 1)); 
    usleep(5000);    //A sample period is 4msec.

    ESP_ERROR_CHECK(i2c_master_receive(dev_handle, data, 2, I2C_MASTER_TIMEOUT_MS));
    if (whichVoltage == READ_BATTERY) 
    {
        *batVolts = (calc_voltage(data[0], data[1]) * BATTERY_DIVIDER_CORRECTION);
    } else {
        *batVolts = calc_voltage(data[0], data[1]);
    }
    return 0;
}
*/

/*
 * @brief i2c master initialization1
 */
/*
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
*/
/*
Calculate actual voltage with 4.096 gain
*/
/*
float calc_voltage(uint8_t lowByte, uint8_t highByte)
{
    return ((lowByte * 256 + highByte) * 0.000125);
}
*/

void app_main(void)
{
    double acc_time = 0.0;
    uint32_t sample_number = 0;
    double_t amp_hours = 0;
    double average_current;
    time_t t;
    struct tm timeinfo;
    char strftime_buf[64];
    uint8_t mac[6];

    esp_efuse_mac_get_default(mac); 
    printf("Unique DevKit ID (MAC): %02X:%02X:%02X:%02X:%02X:%02X\n", 
		       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    init_gpios();

    tm.tm_year = 2026 - 1900;
    tm.tm_mon = 5 - 1;
    tm.tm_mday = 25;
    tm.tm_hour = 8 - 1;
    tm.tm_min = 32;
    tm.tm_sec = 0;  

    
    setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0", 1);
    tzset();
    t = mktime(&tm);
    struct timeval now = { .tv_sec = t};
    settimeofday(&now, NULL);
   
    t = time(NULL);
    struct tm *currentTime = localtime(&t);
    printf("Current Date and Time: %02d/%02d/%04d %02d:%02d:%02d\n",
    currentTime->tm_mday, currentTime->tm_mon + 1, currentTime->tm_year + 1900,
    currentTime->tm_hour, currentTime->tm_min, currentTime->tm_sec);
   
    localtime_r(&t, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Fremont, CA is: %s", strftime_buf);

       //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LDO_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, BATTERY_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
    do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, LDO_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);


    /*
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");
    */

     // Enable task to control LDO enable. 
    xTaskCreatePinnedToCore(
        control_enable,   /* Task function. */
        "control_enable", /* name of task. */
        2048,             /* Stack size of task */
        NULL,             /* parameter of the task */
        5,                /* priority of the task */
        NULL,             /* Task handle to keep track of created task */
        0                 /* tskNO_AFFINITY*/
    );

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(60000 * UPDATE_RATE_DURATION));
        acc_time = acc_time + (UPDATE_RATE_DURATION/60.0);
        sample_number = sample_number + 1;
        average_current = ((regulatorVoltage / 1000.0) / 56.0 * 0.5); // 3v / 60 ohms  50% on time 50% off.
        amp_hours = amp_hours + (average_current * (UPDATE_RATE_DURATION/60.0)); // 5 min / 60 min                                                         
        printf("%lu ETime %.3f Hrs BatV %d mV RegV %d mV %.3f aH\n", sample_number, acc_time, batteryVoltage, regulatorVoltage, amp_hours);
    }

     //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (do_calibration1_chan1) {
        example_adc_calibration_deinit(adc1_cali_chan1_handle);
    }

    /*
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
    */
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

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif


    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
