// dac_i2s_stream.cpp

extern "C" {
#include "driver/i2s.h"
#include "driver/dac.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
}

#include <cmath>
#include <cstdint>
#include <cstring>

#include <Arduino.h>
#include <Adafruit_GFX.h>

static const char* TAG = "DAC_CPP";

#define SAMPLE_RATE     44100
#define BUFFER_SAMPLES  1024
#define DMA_BUF_COUNT   4
#define WDT_TIMEOUT_S   5

static uint16_t* dac_buffer = nullptr;

static void fill_waveform()
{
    for (int i = 0; i < BUFFER_SAMPLES; i++)
    {
        float phase = float(i) / BUFFER_SAMPLES;

        uint8_t left  = uint8_t(127 + 100 * sinf(2 * M_PI * phase)); // DAC2
        uint8_t right = uint8_t(127 + 100 * cosf(2 * M_PI * phase)); // DAC1

        dac_buffer[i] = (uint16_t(left) << 8) | uint16_t(right);
    }
}

static void stream_task(void* arg)
{
    size_t written;

    esp_task_wdt_add(nullptr);

    while (true) {
        i2s_write(I2S_NUM_0, dac_buffer,
                  BUFFER_SAMPLES * sizeof(uint16_t),
                  &written, portMAX_DELAY);

        esp_task_wdt_reset(); // feed watchdog

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void init_i2s()
{
    i2s_config_t config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = BUFFER_SAMPLES,
        .use_apll = false
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &config, 0, nullptr));
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
}

void setup()
{
    dac_buffer = (uint16_t*)heap_caps_malloc(
        BUFFER_SAMPLES * sizeof(uint16_t),
        MALLOC_CAP_DMA
    );

    fill_waveform();
    init_i2s();

    xTaskCreate(stream_task, "stream_task",
                4096, nullptr, 5, nullptr);
}

void loop()
{
    esp_task_wdt_reset(); // feed watchdog

    vTaskDelay(pdMS_TO_TICKS(500));
}