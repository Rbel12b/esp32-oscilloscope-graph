#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>

#define SAMPLE_RATE 44100
#define BUFFER_SAMPLES 1024
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
    static uint16_t *dac_buffer;
};