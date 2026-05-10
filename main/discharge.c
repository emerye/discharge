
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "esp_timer.h"


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

#define LOOP_DELAY_TIME 60000


static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
static gpio_config_t io_conf = {};
static void init_gpios();
static int bat_voltage(int32_t);
static int32_t last_reg_voltage;
static adc_oneshot_unit_handle_t adc1_handle;


//LED task
void led_toggle(void *pvParameters) {
    int current_level = 0;
    while(1) {
        if (current_level == 1) {
            gpio_set_level(GPIO_OUTPUT_IO_18, 0);
            current_level = 0;
        } else {
            gpio_set_level(GPIO_OUTPUT_IO_18, 1);
            current_level = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// LDO task
void control_enable(void *)
{
    //static adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    //bool do_calibration1_chan1;

    bool do_calibration1_chan1 = example_adc_calibration_init(ADC_UNIT_1, LOAD_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &adc1_cali_chan1_handle);

    while (1)
    {
        gpio_set_level(GPIO_OUTPUT_IO_18, 1);

        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LOAD_ADC_CHANNEL, &adc_raw[0][1]));
        if (do_calibration1_chan1)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
            if (voltage[0][1] > 1000)
            {
                last_reg_voltage = voltage[0][1];
            }
        }

        gpio_set_level(GPIO_OUTPUT_IO_18, 0);
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
}

void app_main(void)
{
    double acc_time = 0.0;
    uint32_t sample_number = 0;
    double_t amp_hours = 0;
    double average_current;

    init_gpios();
    // gpio_toggle();

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        //   .clk_src = ADC_ULP_MODE_RISCV,
        //   .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LOAD_ADC_CHANNEL, &config));

    //-------------ADC1 Calibration Init---------------//
    static adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, BATTERY_ADC_CHANNEL, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);

    // Task to flash led
    //   xTaskCreatePinnedToCore(
    //       led_toggle,          /* Task function. */
    //       "led_toggle",        /* name of task. */
    //       2048,                 /* Stack size of task */
    //       NULL,                 /* parameter of the task */
    //       5,                    /* priority of the task */
    //       NULL,                 /* Task handle to keep track of created task */
    //       tskNO_AFFINITY
    //   );

    //
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

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_TIME));

        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATTERY_ADC_CHANNEL, &adc_raw[0][0]));
        if (do_calibration1_chan0)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        }

        acc_time = acc_time + 0.016666666;
        sample_number = sample_number + 1;
        average_current = ((last_reg_voltage / 1000.0) / 60.0 * 0.2) * 6; // 3v / 60 ohms  20% on time 80% off.  6 times a minute.

        amp_hours = amp_hours + (average_current * 0.016666666666); // 1 min / 60 min
        printf("%lu ETime %.3f Hrs BatV %d mV RegV %ld mV %.3f aH\n", sample_number, acc_time, bat_voltage(voltage[0][0]), last_reg_voltage, amp_hours);
    }

    // Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0)
    {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
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


/** Calc battery voltage **/
static int bat_voltage(int32_t adc_reading)
{
   
    double r1 = 680000;  //Resistor connected to battery
    double r2 = 2700000; //Resistor connected to ground
    double offset = 40;  // Adjustment due to input impedence of ADC pin and resistor r1?

    return (int)(((((double) adc_reading)/ r2) * (r1 + r2)) + offset);
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
