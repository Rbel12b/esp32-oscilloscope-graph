#pragma once

#include <Arduino.h>

#define BUFFER_SAMPLES (1024)

#define SAMPLE_RATE (BUFFER_SAMPLES * 32) // 1/32 s -> one full buffer

#define DMA_BUF_LEN 1024
#define DMA_BUF_COUNT 4

static void stream_task_trampoline(void *arg);

class OscilloscopeGraph
{
    friend void stream_task_trampoline(void *arg);
public:
    void init();

private:
    static void init_i2s();
    void fill_waveform();
    void stream_task();

private:
    uint16_t* dac_buffer = nullptr;
};