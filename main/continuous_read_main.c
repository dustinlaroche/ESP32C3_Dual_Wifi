#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/adc.h"
#include "hal/adc_hal.h"
#include "hal/adc_hal_common.h"
#include "hal/soc_hal.h"
#include "adc1_private.h"
#include "common.h"
#include "esp_adc/adc_filter.h"
#include "esp_adc/adc_monitor.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#include "driver/uart.h"



#define UART_NUM UART_NUM_0
#define BAUD_RATE 115200

#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define _EXAMPLE_ADC_UNIT_STR(unit)         #unit
#define EXAMPLE_ADC_UNIT_STR(unit)          _EXAMPLE_ADC_UNIT_STR(unit)
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_0
#define EXAMPLE_ADC_BIT_WIDTH               ADC_BITWIDTH_12

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#else
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#endif

#define EXAMPLE_READ_LEN                    256

#if CONFIG_IDF_TARGET_ESP32
static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
#else
static adc_channel_t channel[2] = {ADC_CHANNEL_0, ADC_CHANNEL_1};
#endif

static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";



static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    /*
    gpio_config_t gpio_config = {
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = 1 << GPIO_NUM_3,
        .intr_type = GPIO_INTR_DISABLE
    };
    */

    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 1000000,
        .conv_mode = EXAMPLE_ADC_CONV_MODE,
        .format = EXAMPLE_ADC_OUTPUT_TYPE,
    };

    /*
    adc_monitor_handle_t monitor_handle = NULL;
    const adc_monitor_config_t monitor_config = {
        .adc_unit = EXAMPLE_ADC_UNIT,
        .channel = channel,
        .h_threshold = 4000,
        .l_threshold = 30
    };
    //adc_monitor_register_callbacks();
    adc_new_continuous_monitor(handle, &monitor_config, &monitor_handle);
    adc_continuous_monitor_enable(monitor_handle);
    */

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}


void app_main(void)
{
    /*
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    const char* message = "Today is Tuesday, May 21, 2024 and here are the results:\r\n";
    uart_write_bytes(UART_NUM, message, strlen(message));
    */

    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

    s_task_handle = xTaskGetCurrentTaskHandle();

    adc_continuous_handle_t handle = NULL;
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    
    adc_iir_filter_handle_t iir_filter_handle = NULL;
    adc_continuous_iir_filter_config_t iir_filter_config[2];
    for (int i = 0; i < 2; i++) {
        iir_filter_config[i].channel = channel[i] & 0x7;
        iir_filter_config[i].unit = EXAMPLE_ADC_UNIT;
        iir_filter_config[i].coeff = ADC_DIGI_IIR_FILTER_COEFF_64;
    }
    adc_new_continuous_iir_filter(handle, &iir_filter_config, &iir_filter_handle);
    adc_continuous_iir_filter_enable(iir_filter_handle);
    

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1) {
        /**
        * This is to show you the way to use the ADC continuous mode driver event callback.
        * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
        * However in this example, the data processing (print) is slow, so you barely block here.
        *
        * Without using this event callback (to notify this task), you can still just call
        * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
        */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        char unit[] = EXAMPLE_ADC_UNIT_STR(EXAMPLE_ADC_UNIT);

        while (1) {
            ret = adc_continuous_read(handle, result, EXAMPLE_READ_LEN, &ret_num, 0);

            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = EXAMPLE_ADC_GET_CHANNEL(p);
                    uint32_t data = EXAMPLE_ADC_GET_DATA(p);
                    if (chan_num < SOC_ADC_CHANNEL_NUM(EXAMPLE_ADC_UNIT)) {
                        if (chan_num == ADC_CHANNEL_0 || chan_num == ADC_CHANNEL_1) {
                            if (chan_num == ADC_CHANNEL_0) {
                                printf("%ld,", data);
                            } else {
                                printf("%ld \n", data);
                            }
                        }
                    } 
                }
                /**
                * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                * usually you don't need this delay (as this task will block for a while).
                */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));

}

