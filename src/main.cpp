#include <Arduino.h>
#include "OscilloscopeGraph.hpp"
#include "esp_task_wdt.h"

OscilloscopeGraph graph;

void setup()
{
    graph.init();
}

void loop()
{
    vTaskDelay(pdMS_TO_TICKS(500));
}