#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include <math.h>
#include <arduinoFFT.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <Arduino.h>
#include <ArduinoJson.h>

// Configuration constants
#define BUFFER_SIZE 256
#define WINDOW_SEC 7
#define MIN_SAMPLE_RATE 10
#define MAX_SAMPLE_RATE 32000
#define SLEEP_DURATION_US 2500000
#define WIFI_RECONNECT_ATTEMPTS 10
#define MQTT_RECONNECT_ATTEMPTS 5

// Task priorities
#define TASK_PRIORITY_SAMPLING 5
#define TASK_PRIORITY_FFT 3
#define TASK_PRIORITY_AGGREGATE 3
#define TASK_PRIORITY_MQTT 2
#define TASK_PRIORITY_PERF 2

// Task stack sizes
#define STACK_SIZE_SAMPLING 4096
#define STACK_SIZE_FFT 8192
#define STACK_SIZE_AGGREGATE 4096
#define STACK_SIZE_MQTT 4096
#define STACK_SIZE_PERF 4096

// WiFi and MQTT credentials
const char* WIFI_SSID = "Najeh's S25 Ultra";
const char* WIFI_PASSWORD = "white1xx";
const char* MQTT_SERVER = "test.mosquitto.org";
const int MQTT_PORT = 1883;
const char* MQTT_CLIENT_ID = "ESP32_FFT_Client_";
const char* MQTT_TOPIC_METRICS = "whitex/sensor/metrics";

// RTC memory
RTC_DATA_ATTR int current_sample_rate = MAX_SAMPLE_RATE;
RTC_DATA_ATTR int cycle_count = 0;
RTC_DATA_ATTR int mqtt_bytes_sent_total = 0;
RTC_DATA_ATTR uint64_t total_latency_us = 0;
RTC_DATA_ATTR int latency_count = 0;
RTC_DATA_ATTR int boot_count = 0;

// FFT result structure
typedef struct {
  float max_frequency;
  float max_magnitude;
  int new_sample_rate;
} FFTResult_t;

// Aggregate result structure
typedef struct {
  float mean;
  float median;
  float mse;
} AggregateResult_t;

// Shared resources
float* sample_buffer_1 = NULL;
float* sample_buffer_2 = NULL;
volatile bool use_buffer_1 = true;
float aggregate_value = 0;
AggregateResult_t aggregate_result = {0, 0, 0}; // Store mean, median, MSE
uint64_t sample_timestamp = 0, publish_timestamp = 0;

// FreeRTOS handles
QueueHandle_t xFFTResultQueue = NULL;
QueueHandle_t xAggregateQueue = NULL;
SemaphoreHandle_t xSampleBufferMutex = NULL;
SemaphoreHandle_t xAggregateMutex = NULL;
SemaphoreHandle_t xTaskCompleteSemaphore = NULL;
SemaphoreHandle_t xSamplingCompleteSemaphore = NULL;
SemaphoreHandle_t xMQTTCompleteSemaphore = NULL; // New semaphore
volatile int tasks_to_complete = 0;
const int TOTAL_TASKS = 5;

// Task handles
TaskHandle_t xSamplingTaskHandle = NULL;
TaskHandle_t xFFTTaskHandle = NULL;
TaskHandle_t xAggregateTaskHandle = NULL;
TaskHandle_t xMQTTTaskHandle = NULL;
TaskHandle_t xPerformanceTaskHandle = NULL;

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Function prototypes
void connect_wifi();
void connect_mqtt();
void reconnect_mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void publish_mqtt_message(const char* topic, const char* payload);
void vSamplingTask(void* pvParameters);
void vFFTTask(void* pvParameters);
void vAggregateTask(void* pvParameters);
void vMQTTTask(void* pvParameters);
void vPerformanceTask(void* pvParameters);
void enter_deep_sleep();

void setup() {
  Serial.begin(115200);
  delay(1000);

  boot_count++;
  Serial.printf("\n\n--- Boot Count: %d ---\n", boot_count);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup Reason: Deep Sleep Timer");
      break;
    default:
      Serial.printf("Wakeup Reason: %d (Power-On, Reset, or Other)\n", wakeup_reason);
      cycle_count = 0;
      mqtt_bytes_sent_total = 0;
      total_latency_us = 0;
      latency_count = 0;
      if (current_sample_rate < MIN_SAMPLE_RATE || current_sample_rate > MAX_SAMPLE_RATE) {
        Serial.printf("Resetting sample rate from %d to %d\n", current_sample_rate, MIN_SAMPLE_RATE);
        current_sample_rate = MIN_SAMPLE_RATE;
      }
      break;
  }
  Serial.printf("Cycle Count: %d\n", cycle_count);
  Serial.printf("Current Sample Rate: %d Hz\n", current_sample_rate);
  Serial.printf("Free heap before setup: %d bytes\n", esp_get_free_heap_size());

  // Initialize watchdog
  Serial.println("Configuring Task Watchdog Timer...");
  esp_err_t wdt_deinit_err = esp_task_wdt_deinit();
  if (wdt_deinit_err == ESP_OK) {
    Serial.println("Task WDT Deinitialized (was running).");
  } else if (wdt_deinit_err == ESP_ERR_INVALID_STATE) {
    Serial.println("Task WDT was not running, no deinit needed.");
  } else {
    Serial.printf("Warning: esp_task_wdt_deinit() error: %s (%d)\n", esp_err_to_name(wdt_deinit_err), wdt_deinit_err);
  }

  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 15000,
    .idle_core_mask = (1 << 2) - 1,
    .trigger_panic = true
  };
  esp_err_t wdt_init_err = esp_task_wdt_init(&wdt_config);
  if (wdt_init_err != ESP_OK) {
    Serial.printf("FATAL: Failed to initialize Task WDT: %s (%d)\n", esp_err_to_name(wdt_init_err), wdt_init_err);
    delay(1000);
    esp_restart();
  }
  Serial.println("Task WDT Initialized.");

  esp_err_t wdt_add_err = esp_task_wdt_add(NULL);
  if (wdt_add_err != ESP_OK) {
    Serial.printf("Note: Main task WDT add error: %s (%d)\n", esp_err_to_name(wdt_add_err), wdt_add_err);
  } else {
    Serial.println("Main task added to WDT.");
  }

  // Allocate buffers
  sample_buffer_1 = (float*)malloc(BUFFER_SIZE * sizeof(float));
  sample_buffer_2 = (float*)malloc(BUFFER_SIZE * sizeof(float));
  if (!sample_buffer_1 || !sample_buffer_2) {
    Serial.println("FATAL: Failed to allocate sample buffers.");
    delay(1000);
    esp_restart();
  }
  memset(sample_buffer_1, 0, BUFFER_SIZE * sizeof(float));
  memset(sample_buffer_2, 0, BUFFER_SIZE * sizeof(float));
  Serial.println("Sample buffers allocated.");

  // Create queues
  xFFTResultQueue = xQueueCreate(1, sizeof(FFTResult_t));
  xAggregateQueue = xQueueCreate(1, sizeof(AggregateResult_t));
  if (!xFFTResultQueue || !xAggregateQueue) {
    Serial.println("FATAL: Failed to create queues.");
    if (sample_buffer_1) free(sample_buffer_1);
    if (sample_buffer_2) free(sample_buffer_2);
    delay(1000);
    esp_restart();
  }
  Serial.println("Queues created.");

  // Create mutexes and semaphores
  xSampleBufferMutex = xSemaphoreCreateMutex();
  xAggregateMutex = xSemaphoreCreateMutex();
  xTaskCompleteSemaphore = xSemaphoreCreateCounting(TOTAL_TASKS, 0);
  xSamplingCompleteSemaphore = xSemaphoreCreateBinary();
  xMQTTCompleteSemaphore = xSemaphoreCreateBinary();
  if (!xSampleBufferMutex || !xAggregateMutex || !xTaskCompleteSemaphore ||
      !xSamplingCompleteSemaphore || !xMQTTCompleteSemaphore) {
    Serial.println("FATAL: Failed to create mutexes/semaphore.");
    if (sample_buffer_1) free(sample_buffer_1);
    if (sample_buffer_2) free(sample_buffer_2);
    if (*sample_buffer_1) free(sample_buffer_1);
    if (sample_buffer_2) free(sample_buffer_2);
    if (xFFTResultQueue) vQueueDelete(xFFTResultQueue);
    if (xAggregateQueue) vQueueDelete(xAggregateQueue);
    delay(1000);
    esp_restart();
  }
  Serial.println("Mutexes and Semaphores created.");

  // Connect to WiFi
  connect_wifi();

  // Connect to MQTT
  if (WiFi.status() == WL_CONNECTED) {
    connect_mqtt();
  } else {
    Serial.println("Skipping MQTT connection (WiFi not connected).");
  }

  Serial.println("Creating tasks...");
  tasks_to_complete = TOTAL_TASKS;

  xTaskCreatePinnedToCore(
    vSamplingTask, "SamplingTask", STACK_SIZE_SAMPLING,
    NULL, TASK_PRIORITY_SAMPLING, &xSamplingTaskHandle, 0);

  xTaskCreatePinnedToCore(
    vMQTTTask, "MQTTTask", STACK_SIZE_MQTT,
    NULL, TASK_PRIORITY_MQTT, &xMQTTTaskHandle, 0);

  xTaskCreatePinnedToCore(
    vFFTTask, "FFTTask", STACK_SIZE_FFT,
    NULL, TASK_PRIORITY_FFT, &xFFTTaskHandle, 1);

  xTaskCreatePinnedToCore(
    vAggregateTask, "AggregateTask", STACK_SIZE_AGGREGATE,
    NULL, TASK_PRIORITY_AGGREGATE, &xAggregateTaskHandle, 1);

  xTaskCreatePinnedToCore(
    vPerformanceTask, "PerformanceTask", STACK_SIZE_PERF,
    NULL, TASK_PRIORITY_PERF, &xPerformanceTaskHandle, 1);

  Serial.printf("Free heap after setup: %d bytes\n", esp_get_free_heap_size());
  Serial.println("--- Setup Complete ---");
}

void loop() {
  esp_task_wdt_reset();

  int completed_count = 0;
  bool all_tasks_completed = true;
  TickType_t start_tick = xTaskGetTickCount();
  TickType_t timeout_ticks = pdMS_TO_TICKS(12000);

  while (completed_count < tasks_to_complete) {
    if (xSemaphoreTake(xTaskCompleteSemaphore, 0) == pdPASS) {
      completed_count++;
    } else {
      if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
        Serial.printf("Loop: Timeout waiting for tasks! Expected %d, got %d.\n", tasks_to_complete, completed_count);
        all_tasks_completed = false;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      esp_task_wdt_reset();
    }
  }

  while (xSemaphoreTake(xTaskCompleteSemaphore, 0) == pdPASS)
    ;

  if (all_tasks_completed) {
    Serial.println("Loop: All tasks completed.");
  } else {
    Serial.println("Loop: Not all tasks completed, entering sleep.");
  }

  enter_deep_sleep();
}

void connect_wifi() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi already connected.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return;
  }

  Serial.printf("Connecting to WiFi SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < WIFI_RECONNECT_ATTEMPTS) {
    delay(1000);
    Serial.print(".");
    attempts++;
    esp_task_wdt_reset();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf("WiFi connection failed after %d attempts!\n", attempts);
  }
}

void connect_mqtt() {
  if (!client.connected()) {
    Serial.printf("Connecting MQTT to %s:%d...\n", MQTT_SERVER, MQTT_PORT);
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setCallback(mqtt_callback);

    String clientId = String(MQTT_CLIENT_ID) + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT connected!");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(client.state());
    }
  } else {
    Serial.println("MQTT already connected.");
  }
}

void reconnect_mqtt() {
  int attempts = 0;
  while (!client.connected() && attempts < MQTT_RECONNECT_ATTEMPTS) {
    attempts++;
    Serial.printf("MQTT reconnect attempt %d/%d...\n", attempts, MQTT_RECONNECT_ATTEMPTS);
    esp_task_wdt_reset();

    String clientId = String(MQTT_CLIENT_ID) + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("MQTT reconnected!");
      return;
    } else {
      Serial.print("MQTT reconnect failed, rc=");
      Serial.println(client.state());
      Serial.println("Retrying in 5 seconds...");
      long startMillis = millis();
      while (millis() - startMillis < 5000) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
      }
    }
  }
  if (!client.connected()) {
    Serial.println("MQTT reconnection failed.");
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  payload[length] = '\0';
  Serial.println((char*)payload);
}

void publish_mqtt_message(const char* topic, const char* payload) {
  if (!client.connected()) {
    Serial.println("MQTT publish failed: Client not connected.");
    return;
  }
  if (client.publish(topic, payload)) {
    int bytes = strlen(topic) + strlen(payload);
    mqtt_bytes_sent_total += bytes;
    Serial.printf("MQTT published to %s: %s\n", topic, payload);
  } else {
    Serial.printf("MQTT publish failed! Topic: %s\n", topic);
  }
}

void vSamplingTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  Serial.println("SamplingTask: Subscribed to WDT");

  int actual_sample_rate = constrain(current_sample_rate, MIN_SAMPLE_RATE, MAX_SAMPLE_RATE);
  if (actual_sample_rate != current_sample_rate) {
    Serial.printf("SamplingTask: Clamped rate from %d to %d Hz\n", current_sample_rate, actual_sample_rate);
  }

  sample_timestamp = esp_timer_get_time();
  float* target_buffer = use_buffer_1 ? sample_buffer_1 : sample_buffer_2;
  const uint32_t sample_period_us = 1000000 / actual_sample_rate;
  bool success = true;
  const float OFFSET = 1.0; // Offset for sine wave

  Serial.printf("SamplingTask started at %llu, rate %d Hz\n", sample_timestamp, actual_sample_rate);

  for (int i = 0; i < BUFFER_SIZE && success; i++) {
    uint64_t start_sample_time = esp_timer_get_time();
    float t = (float)start_sample_time / 1000000.0;
    float signal = 2.0 * sin(2.0 * M_PI * 150.0 * t) + 4.0 * sin(2.0 * M_PI * 100.0 * t) + OFFSET;

    if (xSampleBufferMutex && xSemaphoreTake(xSampleBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      target_buffer[i] = signal;
      xSemaphoreGive(xSampleBufferMutex);
    } else {
      Serial.println("SamplingTask: Buffer mutex timeout");
      success = false;
    }

    uint64_t computation_time = esp_timer_get_time() - start_sample_time;
    if (computation_time < sample_period_us) {
      delayMicroseconds(sample_period_us - computation_time);
    } else {
      Serial.printf("SamplingTask: WARN - Computation time (%llu us) >= Period (%u us)\n", computation_time, sample_period_us);
    }
    esp_task_wdt_reset();
  }

  if (success && xSampleBufferMutex && xSemaphoreTake(xSampleBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    use_buffer_1 = !use_buffer_1;
    xSemaphoreGive(xSampleBufferMutex);
    Serial.println("SamplingTask: Buffer swapped");
  } else if (success) {
    Serial.println("SamplingTask: Swap mutex timeout");
    success = false;
  }

  Serial.printf("SamplingTask complete (%s)\n", success ? "Success" : "Failed");

  if (xSamplingCompleteSemaphore) {
    xSemaphoreGive(xSamplingCompleteSemaphore);
    Serial.println("SamplingTask: Signaled FFT task");
  }

  if (xTaskCompleteSemaphore) {
    xSemaphoreGive(xTaskCompleteSemaphore);
  }

  esp_task_wdt_delete(NULL);
  Serial.println("SamplingTask: About to delete itself");
  vTaskDelete(NULL);
}

void vFFTTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  Serial.println("FFTTask: Subscribed to WDT");

  int rate_data_was_sampled_at = constrain(current_sample_rate, MIN_SAMPLE_RATE, MAX_SAMPLE_RATE);
  uint64_t start_time = esp_timer_get_time();
  Serial.printf("FFTTask started at %llu, rate %d Hz\n", start_time, rate_data_was_sampled_at);

  bool success = true;
  FFTResult_t fftResult = { 0, 0, rate_data_was_sampled_at };

  vTaskDelay(pdMS_TO_TICKS(100));
  Serial.println("FFTTask: Waiting for sampling signal");

  if (xSamplingCompleteSemaphore && xSemaphoreTake(xSamplingCompleteSemaphore, pdMS_TO_TICKS(10000)) == pdPASS) {
    Serial.println("FFTTask: Received sampling signal");
  } else {
    Serial.println("FFTTask: Timeout waiting for sampling signal!");
    success = false;
  }

  if (success) {
    float* source_buffer = use_buffer_1 ? sample_buffer_2 : sample_buffer_1;
    double* vReal = (double*)malloc(BUFFER_SIZE * sizeof(double));
    double* vImag = (double*)malloc(BUFFER_SIZE * sizeof(double));
    ArduinoFFT<double>* FFT = NULL;

    if (!vReal || !vImag) {
      Serial.println("FFTTask: Memory allocation failed!");
      if (vReal) free(vReal);
      if (vImag) free(vImag);
      success = false;
    }

    if (success) {
      FFT = new ArduinoFFT<double>(vReal, vImag, BUFFER_SIZE, (double)rate_data_was_sampled_at);
      if (!FFT) {
        Serial.println("FFTTask: FFT instance creation failed!");
        free(vReal);
        free(vImag);
        success = false;
      }
    }

    if (success && xSampleBufferMutex && xSemaphoreTake(xSampleBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      memcpy(vReal, source_buffer, BUFFER_SIZE * sizeof(double));
      memset(vImag, 0, BUFFER_SIZE * sizeof(double));
      xSemaphoreGive(xSampleBufferMutex);
    } else if (success) {
      Serial.println("FFTTask: Buffer mutex timeout!");
      success = false;
    }

    if (success) {
      FFT->windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT->compute(FFT_FORWARD);
      FFT->complexToMagnitude();

      float max_freq = 0;
      float max_magnitude = 0;
      for (int i = 1; i < BUFFER_SIZE / 2; i++) {
        if (vReal[i] > max_magnitude) {
          max_magnitude = vReal[i];
          max_freq = (float)i * rate_data_was_sampled_at / BUFFER_SIZE;
        }
      }
      Serial.printf("FFTTask: Max freq = %.2f Hz, Mag = %.2f\n", max_freq, max_magnitude);

      int new_rate = (int)(2.2 * max_freq + 0.5);
      new_rate = constrain(new_rate, MIN_SAMPLE_RATE, MAX_SAMPLE_RATE);
      if (new_rate != rate_data_was_sampled_at) {
        Serial.printf("FFTTask: Adjusting rate to %d Hz\n", new_rate);
      }
      current_sample_rate = new_rate;

      fftResult.max_frequency = max_freq;
      fftResult.max_magnitude = max_magnitude;
      fftResult.new_sample_rate = current_sample_rate;
    }

    if (FFT) delete FFT;
    if (vReal) free(vReal);
    if (vImag) free(vImag);
  }

  if (xFFTResultQueue) {
    if (xQueueOverwrite(xFFTResultQueue, &fftResult) == pdPASS) {
      Serial.println("FFTTask: Sent result to Aggregate queue");
    } else {
      Serial.println("FFTTask: Queue send failed!");
      success = false;
    }
  } else {
    Serial.println("FFTTask: Queue null!");
    success = false;
  }

  Serial.printf("FFTTask complete (%s)\n", success ? "Success" : "Failed");

  if (xTaskCompleteSemaphore) {
    xSemaphoreGive(xTaskCompleteSemaphore);
  }

  esp_task_wdt_delete(NULL);
  vTaskDelete(NULL);
}

void vAggregateTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  Serial.println("AggregateTask: Subscribed to WDT");

  int rate_data_was_sampled_at = constrain(current_sample_rate, MIN_SAMPLE_RATE, MAX_SAMPLE_RATE);
  uint64_t start_time = esp_timer_get_time();
  Serial.printf("AggregateTask started at %llu\n", start_time);

  bool success = true;
  FFTResult_t fftResult;
  AggregateResult_t aggResult = {0, 0, 0};

  Serial.println("AggregateTask: Waiting for FFT result");
  if (xFFTResultQueue && xQueueReceive(xFFTResultQueue, &fftResult, pdMS_TO_TICKS(12000)) == pdPASS) {
    Serial.println("AggregateTask: Received FFT result");
  } else {
    Serial.println("AggregateTask: FFT queue timeout!");
    success = false;
  }

  if (success) {
    float* source_buffer = use_buffer_1 ? sample_buffer_2 : sample_buffer_1;
    int num_samples_in_window = rate_data_was_sampled_at * WINDOW_SEC;
    int num_samples_to_average = min(num_samples_in_window, BUFFER_SIZE);

    if (num_samples_to_average <= 0) {
      Serial.printf("AggregateTask: Invalid num_samples_to_average (%d)\n", num_samples_to_average);
      aggResult.mean = 0;
      aggResult.median = 0;
      aggResult.mse = 0;
    } else {
      float sum = 0;
      float* temp_buffer = (float*)malloc(num_samples_to_average * sizeof(float));
      if (!temp_buffer) {
        Serial.println("AggregateTask: Memory allocation failed for temp buffer!");
        success = false;
      }

      if (success && xSampleBufferMutex && xSemaphoreTake(xSampleBufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int start_index = BUFFER_SIZE - num_samples_to_average;
        for (int i = 0; i < num_samples_to_average; i++) {
          temp_buffer[i] = source_buffer[start_index + i];
          sum += temp_buffer[i];
        }
        xSemaphoreGive(xSampleBufferMutex);

        aggResult.mean = sum / num_samples_to_average;

        for (int i = 0; i < num_samples_to_average - 1; i++) {
          for (int j = 0; j < num_samples_to_average - i - 1; j++) {
            if (temp_buffer[j] > temp_buffer[j + 1]) {
              float temp = temp_buffer[j];
              temp_buffer[j] = temp_buffer[j + 1];
              temp_buffer[j + 1] = temp;
            }
          }
        }
        if (num_samples_to_average % 2 == 0) {
          aggResult.median = (temp_buffer[num_samples_to_average / 2 - 1] + temp_buffer[num_samples_to_average / 2]) / 2.0;
        } else {
          aggResult.median = temp_buffer[num_samples_to_average / 2];
        }

        float sum_squared_error = 0;
        for (int i = 0; i < num_samples_to_average; i++) {
          float error = temp_buffer[i] - aggResult.mean;
          sum_squared_error += error * error;
        }
        aggResult.mse = sum_squared_error / num_samples_to_average;

        Serial.print("AggregateTask: Samples: ");
        for (int i = 0; i < num_samples_to_average; i++) {
          Serial.print(temp_buffer[i]);
          Serial.print(" ");
        }
        Serial.println();

        free(temp_buffer);
      } else {
        Serial.println("AggregateTask: Buffer mutex timeout!");
        success = false;
        aggResult.mean = 0;
        aggResult.median = 0;
        aggResult.mse = 0;
      }
    }

    if (success && xAggregateMutex && xSemaphoreTake(xAggregateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      aggregate_value = aggResult.mean;
      xSemaphoreGive(xAggregateMutex);
    } else if (success) {
      Serial.println("AggregateTask: Aggregate mutex timeout");
    }

    if (xAggregateQueue) {
      if (xQueueOverwrite(xAggregateQueue, &aggResult) == pdPASS) {
        Serial.println("AggregateTask: Sent aggregate results to MQTT queue");
      } else {
        Serial.println("AggregateTask: Queue send failed!");
        success = false;
      }
    } else {
      Serial.println("AggregateTask: Queue null!");
      success = false;
    }
  }

  Serial.printf("AggregateTask complete (%s), Mean = %.2f, Median = %.2f, MSE = %.2f\n",
                success ? "Success" : "Failed", aggResult.mean, aggResult.median, aggResult.mse);

  if (xTaskCompleteSemaphore) {
    xSemaphoreGive(xTaskCompleteSemaphore);
  }

  esp_task_wdt_delete(NULL);
  vTaskDelete(NULL);
}

void vMQTTTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  Serial.println("MQTTTask: Subscribed to WDT");

  uint64_t start_time = esp_timer_get_time();
  Serial.printf("MQTTTask started at %llu\n", start_time);

  bool success = true;
  AggregateResult_t aggResult = {0, 0, 0};

  Serial.println("MQTTTask: Waiting for aggregate values");
  if (xAggregateQueue) {
    if (xQueueReceive(xAggregateQueue, &aggResult, pdMS_TO_TICKS(5000)) == pdPASS) {
      Serial.println("MQTTTask: Received aggregate results");
      Serial.printf("MQTTTask: Mean=%.2f, Median=%.2f, MSE=%.2f\n", aggResult.mean, aggResult.median, aggResult.mse);
    } else {
      Serial.println("MQTTTask: Queue timeout!");
      success = false;
    }
  } else {
    Serial.println("MQTTTask: Queue null!");
    success = false;
  }

  if (success) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("MQTTTask: WiFi disconnected, reconnecting...");
      connect_wifi();
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        Serial.println("MQTTTask: MQTT disconnected, reconnecting...");
        reconnect_mqtt();
      }

      if (client.connected()) {
        // Store aggregate results globally
        if (xAggregateMutex && xSemaphoreTake(xAggregateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
          aggregate_result.mean = aggResult.mean;
          aggregate_result.median = aggResult.median;
          aggregate_result.mse = aggResult.mse;
          xSemaphoreGive(xAggregateMutex);
          Serial.printf("MQTTTask: Stored Mean=%.2f, Median=%.2f, MSE=%.2f\n",
                        aggResult.mean, aggResult.median, aggResult.mse);
          xSemaphoreGive(xMQTTCompleteSemaphore); // Signal PerformanceTask
        } else {
          Serial.println("MQTTTask: Aggregate mutex timeout");
          success = false;
        }

        publish_timestamp = esp_timer_get_time();
        if (sample_timestamp > 0) {
          uint64_t latency_us = publish_timestamp - sample_timestamp;
          total_latency_us += latency_us;
          latency_count++;
          Serial.printf("MQTTTask: Latency: %llu us\n", latency_us);
        }

        client.loop();
      } else {
        Serial.println("MQTTTask: Not connected.");
        success = false;
      }
    } else {
      Serial.println("MQTTTask: WiFi not connected.");
      success = false;
    }
  }

  Serial.printf("MQTTTask complete (%s)\n", success ? "Success" : "Failed");

  if (xTaskCompleteSemaphore) {
    xSemaphoreGive(xTaskCompleteSemaphore);
  }

  esp_task_wdt_delete(NULL);
  vTaskDelete(NULL);
}

void vPerformanceTask(void* pvParameters) {
  esp_task_wdt_add(NULL);
  Serial.println("PerformanceTask: Subscribed to WDT");

  uint64_t start_time = esp_timer_get_time();
  Serial.printf("PerformanceTask started at %llu\n", start_time);

  bool success = true;
  AggregateResult_t aggResult = {0, 0, 0};
  float avg_latency_ms = latency_count > 0 ? (float)(total_latency_us / latency_count) / 1000.0 : 0;

  // Wait for MQTTTask to complete
  Serial.println("PerformanceTask: Waiting for MQTTTask");
  if (xMQTTCompleteSemaphore && xSemaphoreTake(xMQTTCompleteSemaphore, pdMS_TO_TICKS(5000)) == pdPASS) {
    Serial.println("PerformanceTask: MQTTTask completed");
  } else {
    Serial.println("PerformanceTask: Timeout waiting for MQTTTask!");
    success = false;
  }

  // Retrieve aggregate results
  if (xAggregateMutex && xSemaphoreTake(xAggregateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    aggResult = aggregate_result;
    Serial.printf("PerformanceTask: Read Mean=%.2f, Median=%.2f, MSE=%.2f\n",
                  aggResult.mean, aggResult.median, aggResult.mse);
    xSemaphoreGive(xAggregateMutex);
  } else {
    Serial.println("PerformanceTask: Aggregate mutex timeout");
    success = false;
  }

  if (success && WiFi.status() == WL_CONNECTED && client.connected()) {
    // Create JSON document
    StaticJsonDocument<256> doc;
    doc["mean"] = round(aggResult.mean);
    doc["median"] = round(aggResult.median);
    doc["mse"] = round(aggResult.mse);
    doc["cycle"] = cycle_count;
    doc["latency_ms"] = round(avg_latency_ms * 10) / 10.0;
    doc["rate"] = current_sample_rate;
    doc["mqtt_bytes"] = mqtt_bytes_sent_total;
    doc["heap"] = esp_get_free_heap_size();

    // Serialize JSON to string
    char metrics_buffer[256];
    size_t len = serializeJson(doc, metrics_buffer, sizeof(metrics_buffer));

    if (len > 0) {
      Serial.printf("PerformanceTask: Publishing JSON: %s\n", metrics_buffer);
      publish_mqtt_message(MQTT_TOPIC_METRICS, metrics_buffer);
    } else {
      Serial.println("PerformanceTask: JSON serialization failed!");
      success = false;
    }
  } else {
    Serial.println("PerformanceTask: Cannot publish (not connected)");
    success = false;
  }

  Serial.printf("PerformanceTask complete (%s)\n", success ? "Success" : "Failed");

  if (xTaskCompleteSemaphore) {
    xSemaphoreGive(xTaskCompleteSemaphore);
  }

  esp_task_wdt_delete(NULL);
  vTaskDelete(NULL);
}

void enter_deep_sleep() {
  cycle_count++;
  Serial.printf("Cycle %d finished. Sleeping for %.2f seconds...\n", cycle_count, (float)SLEEP_DURATION_US / 1000000.0);
  Serial.flush();

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  esp_deep_sleep_start();
}