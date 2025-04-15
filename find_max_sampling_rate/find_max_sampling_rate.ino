#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/adc.h>
#include <esp_timer.h>

#define TEST_SAMPLES 10000
float test_buffer[TEST_SAMPLES];
SemaphoreHandle_t xBufferMutex;

void sampling_test_task(void *pvParameters) {
    int index = 0;
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    
    uint64_t start_time = esp_timer_get_time();
    
    while (index < TEST_SAMPLES) {
        float signal = adc1_get_raw(ADC1_CHANNEL_0) / 4096.0 * 3.3;
        if (xSemaphoreTake(xBufferMutex, portMAX_DELAY) == pdTRUE) {
            test_buffer[index++] = signal;
            xSemaphoreGive(xBufferMutex);
        }
    }
    
    uint64_t end_time = esp_timer_get_time();
    float duration_s = (end_time - start_time) / 1000000.0;
    float sample_rate = TEST_SAMPLES / duration_s;
    
    Serial.printf("Sampled %d samples in %.6f seconds\n", TEST_SAMPLES, duration_s);
    Serial.printf("Maximum sampling rate: %.2f Hz\n", sample_rate);
    
    vTaskDelete(NULL);
}

void setup() {
    Serial.begin(115200);
    xBufferMutex = xSemaphoreCreateMutex();
    xTaskCreate(sampling_test_task, "SamplingTest", 4096, NULL, 5, NULL);
}

void loop() {}
