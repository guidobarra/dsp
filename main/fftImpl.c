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
#include <math.h>

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

static const char *TAG = "ADC_FFT";

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

// -------------------- IMPLEMENTATION FFT--------------------
// bit-reversal
void rearrange(float data_re[], float data_im[], int N) {
    int j = 0;
    for (int i = 1; i < N; i++) {
        int bit = N >> 1;
        while (j & bit) {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;

        if (i < j) {
            // Intercambiar partes reales
            float temp = data_re[i];
            data_re[i] = data_re[j];
            data_re[j] = temp;

            // Intercambiar partes imaginarias
            temp = data_im[i];
            data_im[i] = data_im[j];
            data_im[j] = temp;
        }
    }
}

// cálcula la FFT
void compute(float data_re[], float data_im[], int N) {
    for (int step = 2; step <= N; step *= 2) {
        int half_step = step / 2;
        float angle_step = -2.0 * M_PI / step;

        for (int group = 0; group < N; group += step) {
            for (int pair = 0; pair < half_step; pair++) {
                int match = group + pair + half_step;
                int i = group + pair;

                // Calcular factores de giro (twiddle factors)
                float angle = angle_step * pair;
                float twiddle_re = cos(angle);
                float twiddle_im = sin(angle);

                // Multiplicación compleja: temp = data[match] * twiddle
                float temp_re = data_re[match] * twiddle_re - data_im[match] * twiddle_im;
                float temp_im = data_re[match] * twiddle_im + data_im[match] * twiddle_re;

                // Operaciones butterfly
                data_re[match] = data_re[i] - temp_re;
                data_im[match] = data_im[i] - temp_im;
                data_re[i] = data_re[i] + temp_re;
                data_im[i] = data_im[i] + temp_im;
            }
        }
    }
}

void fft(float data_re[], float data_im[], int N) {
    rearrange(data_re, data_im, N);
    compute(data_re, data_im, N);
}

void ifft(float data_re[], float data_im[], int N) {
    // Conjugar la entrada
    for (int i = 0; i < N; i++) {
        data_im[i] = -data_im[i];
    }

    // Aplicar FFT
    fft(data_re, data_im, N);

    // Conjugar la salida y escalar por 1/N
    for (int i = 0; i < N; i++) {
        data_im[i] = -data_im[i];
        data_re[i] /= N;
        data_im[i] /= N;
    }
}
// -------------------- IMPLEMENTATION FFT--------------------

void print_complex_array(float data_re[], float data_im[], int N) {
    for (int i = 0; i < N; i++) {
        printf("%10.4f	%10.4f\n", data_re[i], data_im[i]);
    }
}


// -------------------- Main Loop --------------------
void app_main(void)
{
    uint8_t result[ADC_FRAME_SIZE] = {0};
    uint32_t ret_num = 0;

    dac_init();
    continuous_adc_init();

    float input_real[64];
    float input_image[64];
    int cantSample = 64;
    int count = 0;

    while (1)
    {
        if (flag)
        {
            flag = 0;
            adc_continuous_read(ADC_handle, result, ADC_FRAME_SIZE, &ret_num, 0);
            adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[0];
            uint32_t data = ADC_GET_DATA(p); // 12 bits (0–4095)

            float normalized_sample = (float)data / 4095.0f;

            if(count < cantSample) {
                input_real[count] = normalized_sample;
                input_image[count] = 0.0f ;
            }

            if(count == (cantSample-1) ) {
                ESP_LOGI(TAG, "Initial samples fft:");
                print_complex_array(input_real, input_image, cantSample);

                fft(input_real, input_image, cantSample);
                ESP_LOGI(TAG, "Result fft:");
                print_complex_array(input_real, input_image, cantSample);

                count = 0;
            } else {
                count++;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(ADC_handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(ADC_handle));
}