#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/gpio.h"
#include "driver/dac_oneshot.h"
#include "driver/dac_continuous.h"
#include "hal/misc.h"
#include <sys/types.h>

#define ADC_UNIT                    ADC_UNIT_1
#define ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define ADC_ATTEN                   ADC_ATTEN_DB_12
#define ADC_BIT_WIDTH               CONFIG_SOC_ADC_DIGI_MAX_BITWIDTH
#define ADC_BUFFER_SIZE             1024
#define ADC_FRAME_SIZE              4
#define ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p_data)     ((p_data)->type1.channel)
#define ADC_GET_DATA(p_data)        ((p_data)->type1.data)
#define ADC_FRECUENCY_HZ            50000

#define DAC_CHAN                    DAC_CHAN_0

static adc_channel_t channel[2] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
dac_oneshot_handle_t DAC_handle;
adc_continuous_handle_t ADC_handle = NULL;

int flag = 0;

// Callback cuando se genera un marco de conversión
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    flag = 1;
    return true;
}

static void continuous_adc_init()
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = ADC_BUFFER_SIZE,
        .conv_frame_size = ADC_FRAME_SIZE,
    };
    adc_continuous_new_handle(&adc_config, &ADC_handle);
    int channel_num = sizeof(channel) / sizeof(adc_channel_t);

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = ADC_FRECUENCY_HZ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = ADC_UNIT;
        adc_pattern[i].bit_width = ADC_BIT_WIDTH;
    }
    dig_cfg.adc_pattern = adc_pattern;

    adc_continuous_config(ADC_handle, &dig_cfg);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(ADC_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(ADC_handle));
}

void dac_init(void)
{
    dac_oneshot_config_t dac_config = {
        .chan_id = DAC_CHAN,
    };
    dac_oneshot_new_channel(&dac_config, &DAC_handle);
}

// -------------------- FIR --------------------
#define FIR_ORDER 6

//PROMEDIO
//const float fir_coeffs[FIR_ORDER] = {0.25f, 0.25f, 0.25f, 0.25f};

//FPB - fs 18KHz - fc 2.5k
//const float fir_coeffs[FIR_ORDER] = {0.0901728810f, 0.1772164179f, 0.23261070097f, 0.23261070097f, 0.1772164179f, 0.0901728810f};

//FPA - fs 18KHz - fc 7,5k
//const float fir_coeffs[FIR_ORDER] = {-0.1404561200f, 0.1713683602f, -0.1881755197f, 0.1881755197f, -0.1713683602f, 0.1404561200f};

//FPBanda - fs 18KHz - fci 2.5k fcs 7.5 khz
//const float fir_coeffs[FIR_ORDER] = {-0.0643673738f, -0.3203116585f, 0.3120580190f, 0.3120580190f, -0.3203116585f, -0.0643673738f};

//3.1.1, FPB - fs 18KHz - fc 4.5k (ORDEN 5)
//const float fir_coeffs[FIR_ORDER] = {-0.0882352941f, 0.1470588235f, 0.4411764705f, 0.4411764705f, 0.1470588235f, -0.0882352941f};

//3.1.2, FPB - fs 18KHz - fc 4.5k (ORDEN 10)
//const float fir_coeffs[FIR_ORDER] = {0.0605303122f, 0.0f, -0.1008838537f, 0.0f, 0.3026516118f, 0.4754039606f, 0.3026516118f, 0.0f, -0.1008838537f, 0.0f, 0.0605303122f};

//3.1.3 hann y black en ese orden, FPB - fs 18KHz - fc 4.5k
//const float fir_coeffs[FIR_ORDER] = {-0.0077763127f, 0.0644546455f, 0.4433216671f, 0.4433216671f, 0.0644546455f, -0.0077763127f};
const float fir_coeffs[FIR_ORDER] = {0.0f, 0.0365241197f, 0.4634758802f, 0.4634758802f, 0.0365241197f, 0.0f};

float fir_buffer[FIR_ORDER] = {0.0f};

float fir_filter(float new_sample)
{
    // Desplazar buffer
    for (int i = FIR_ORDER - 1; i > 0; i--)
        fir_buffer[i] = fir_buffer[i - 1];
    fir_buffer[0] = new_sample;

    // Convolución
    float result = 0.0f;
    for (int i = 0; i < FIR_ORDER; i++)
        result += fir_coeffs[i] * fir_buffer[i];

    return result;
}
// -------------------- FIR --------------------


// -------------------- Main Loop --------------------
void app_main(void)
{
    uint8_t result[ADC_FRAME_SIZE] = {0};
    uint32_t ret_num = 0;

    dac_init();
    continuous_adc_init();
    bool isFilterPB = true;

    while (1)
    {
        if (flag)
        {
            flag = 0;
            adc_continuous_read(ADC_handle, result, ADC_FRAME_SIZE, &ret_num, 0);
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[0];

            uint32_t data = ADC_GET_DATA(p);

            // 1️⃣ Normalizar a 0-1
            float normalized_sample = (float)data / 4095.0f;
             
            // 2️⃣ Aplicar FIR
            float filtered_sample = fir_filter(normalized_sample);

            if (!isFilterPB)
                filtered_sample = filtered_sample + 0.5f;

            // 3️⃣ Saturar
            if (filtered_sample < 0.0f) filtered_sample = 0.0f;
            if (filtered_sample > 1.0f) filtered_sample = 1.0f;

            // 4️⃣ Escalar a DAC 0-255 (8 bits)
            uint8_t dac_value = (uint8_t)(filtered_sample * 255.0f);
        
            // 5️⃣ Escribir al DAC
            dac_oneshot_output_voltage(DAC_handle, dac_value);
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(ADC_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(ADC_handle));
}