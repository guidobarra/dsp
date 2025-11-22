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

// -------------------- FILTER IIR --------------------
#define IIR_ORDER 6

float iir_coeffs_x[IIR_ORDER] = {0.0528f, 0.2640f, 0.5279f, 0.5279f, 0.2640f, 0.0528f};
float iir_coeffs_y[IIR_ORDER] = {1.0f, 0.0f, 0.6335f, 0.0f, 0.0557f, 0.0f};
float x_buffer[IIR_ORDER] = {0};   // entradas
float y_buffer[IIR_ORDER] = {0};   // salidas

float iir_filter(float new_sample)
{
    // Desplazar entradas
    for (int i = IIR_ORDER - 1; i > 0; i--)
        x_buffer[i] = x_buffer[i - 1];
    x_buffer[0] = new_sample;

    // Desplazar salidas
    for (int i = IIR_ORDER - 1; i > 0; i--)
        y_buffer[i] = y_buffer[i - 1];

    float y = 0.0f;

    // Parte FIR: sumatoria b[k] * x[n-k]
    for (int i = 0; i < IIR_ORDER; i++)
        y += iir_coeffs_x[i] * x_buffer[i];

    // Parte IIR: resta a[k] * y[n-k]
    for (int i = 1; i < IIR_ORDER; i++)
        y -= iir_coeffs_y[i] * y_buffer[i - 1];

    y_buffer[0] = y;

    return y;
}

#define NUM_SOS 3

// Coeficientes B (numerador) de cada SOS: b0, b1, b2
float iir_sos_coeffs_x[NUM_SOS][3] = {
    {1.0f,  2.0f, 1.0f},  // Sección 1
    {1.0f,  2.0f, 1.0f},  // Sección 2
    {1.0f,  1.0f, 0.0f}   // Sección 3
};

// Coeficientes A (denominador): a0, a1, a2
float iir_sos_coeffs_y[NUM_SOS][3] = {
    {1.0f,  0.0f, 0.5279f},
    {1.0f,  0.0f, 0.1056f},
    {1.0f,  0.0f, 0.0f}
};

// Ganancias G
float G[NUM_SOS] = {0.3820f, 0.2764f, 0.5000f};

// Buffers de cada sección
float x_sos_buffer[NUM_SOS][3] = {0};
float y_sos_buffer[NUM_SOS][3] = {0};

float iir_sos_filter(float input_sample)
{
    float x = input_sample;
    float y;

    for (int s = 0; s < NUM_SOS; s++)
    {
        float b0 = iir_sos_coeffs_x[s][0];
        float b1 = iir_sos_coeffs_x[s][1];
        float b2 = iir_sos_coeffs_x[s][2];

        float a0 = iir_sos_coeffs_y[s][0];
        float a1 = iir_sos_coeffs_y[s][1];
        float a2 = iir_sos_coeffs_y[s][2];

        // Recuperar historial
        float x0 = x;
        float x1 = x_sos_buffer[s][0];
        float x2 = x_sos_buffer[s][1];

        float y1 = y_sos_buffer[s][0];
        float y2 = y_sos_buffer[s][1];

        // Ecuación de diferencia
        y = (b0*x0 + b1*x1 + b2*x2
            - a1*y1 - a2*y2) / a0;

        // Guardar valores en buffers
        x_sos_buffer[s][1] = x1;
        x_sos_buffer[s][0] = x0;

        y_sos_buffer[s][1] = y1;
        y_sos_buffer[s][0] = y;

        // Aplicar ganancia de la sección
        y *= G[s];

        // La salida se convierte en entrada de la siguiente sección
        x = y;
    }

    return y;
}
// -------------------- FILTER IIR --------------------

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
            float filtered_sample = iir_filter(normalized_sample);

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