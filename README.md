# ESP32 Sensor Data Processing Application

This Project describes an ESP32-based application for sampling sensor data, performing Fast Fourier Transform (FFT) analysis, aggregating statistical metrics, publishing results via MQTT, and monitoring system performance. Built with FreeRTOS for task management, it optimizes resource usage and employs deep sleep for power efficiency. The application is designed for IoT use cases like signal analysis or environmental monitoring, with support for visualization in Grafana, sending data to MQTT broker using Wifi and using LoRaWAN via The Things Network (TTN).

## Table of Contents
- [ESP32 Sensor Data Processing Application](#esp32-sensor-data-processing-application)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [What the Code Does](#what-the-code-does)
  - [System Architecture](#system-architecture)
    - [Tasks](#tasks)
    - [Shared Resources](#shared-resources)
    - [RTC Memory](#rtc-memory)
  - [Phases of Operation](#phases-of-operation)
  - [Data Flow](#data-flow)
  - [Design Choices and Rationale](#design-choices-and-rationale)
  - [Configuration Parameters](#configuration-parameters)
  - [Dependencies](#dependencies)
  - [Error Handling and Reliability](#error-handling-and-reliability)
  - [Grafana Visualization](#grafana-visualization)
  - [Hands-On Walkthrough](#hands-on-walkthrough)
    - [Prerequisites](#prerequisites)
    - [Step-by-Step Setup](#step-by-step-setup)
    - [Setting Up LoRaWAN with TTN](#setting-up-lorawan-with-ttn)
    - [Troubleshooting](#troubleshooting)

## Overview
The application runs on an ESP32 microcontroller, utilizing its dual-core processor and FreeRTOS to manage concurrent tasks. It simulates sensor data with a synthetic signal, processes it to extract frequency components, computes statistics, and publishes results to an MQTT broker over WiFi. Performance metrics (e.g., latency, heap usage) are tracked, and deep sleep reduces power consumption between cycles. The system supports visualization of aggregated data in Grafana and adapted for LoRaWAN connectivity via TTN for low-power, long-range communication.

## What the Code Does
The code implements a pipeline for processing and transmitting sensor data:

1. **Data Sampling**:
   - Generates a synthetic signal (100 Hz + 150 Hz sine waves, 1.0 offset) to mimic sensor input.
   - Stores samples in double buffers to prevent data conflicts.

2. **FFT Analysis**:
   - Uses `ArduinoFFT` to compute the dominant frequency.
   - Adjusts sampling rate dynamically (2.2x max frequency, Nyquist criterion).

3. **Data Aggregation**:
   - Calculates mean, median, and mean squared error (MSE) over a 7-second window.
   - Prepares metrics for transmission.

4. **MQTT Publishing**:
   - Connects to WiFi and an MQTT broker (`test.mosquitto.org`).
   - Publishes metrics to `whitex/sensor/metrics`.

5. **Performance Monitoring**:
   - Tracks latency, heap usage, and MQTT data volume.
   - Publishes JSON-formatted metrics via MQTT.

6. **Power Management**:
   - Enters deep sleep for 2.5 seconds per cycle.
   - Preserves state in RTC memory.

**Key Features**:
- Concurrent task execution with FreeRTOS.
- Double-buffering for seamless data handling.
- Dynamic sampling rate for efficiency.
- Robust error handling with watchdog timer.
- Support for Grafana visualization and LoRaWAN integration.

## System Architecture
The application uses five FreeRTOS tasks, pinned to the ESP32’s dual cores for load balancing. FreeRTOS primitives (queues, mutexes, semaphores) ensure synchronization and data integrity.

### Tasks
- **Sampling Task** (`vSamplingTask`, Core 0, Priority 5): Generates signal data.
- **FFT Task** (`vFFTTask`, Core 1, Priority 3): Extracts frequency components.
- **Aggregate Task** (`vAggregateTask`, Core 1, Priority 3): Computes statistics.
- **MQTT Task** (`vMQTTTask`, Core 0, Priority 2): Handles WiFi/MQTT communication.
- **Performance Task** (`vPerformanceTask`, Core 1, Priority 2): Publishes metrics.

### Shared Resources
- **Buffers**: `sample_buffer_1`, `sample_buffer_2` (256 samples each), protected by `xSampleBufferMutex`.
- **Queues**: 
  - `xFFTResultQueue`: Passes `FFTResult_t` (frequency, magnitude, rate).
  - `xAggregateQueue`: Passes `AggregateResult_t` (mean, median, MSE).
- **Mutexes**: `xSampleBufferMutex`, `xAggregateMutex`.
- **Semaphores**: `xSamplingCompleteSemaphore`, `xMQTTCompleteSemaphore`, `xTaskCompleteSemaphore`.

### RTC Memory
Stores `current_sample_rate`, `cycle_count`, `mqtt_bytes_sent_total`, `total_latency_us`, `latency_count`, and `boot_count` to persist across sleep cycles.

## Phases of Operation
The application executes three phases per cycle:

1. **Initialization (Setup Phase)**:
   - Initializes serial output (115200 baud).
   - Checks wakeup reason to reset state if needed.
   - Configures WDT (15-second timeout).
   - Allocates buffers and FreeRTOS primitives.
   - Connects to WiFi/MQTT.
   - Launches tasks.

2. **Processing (Task Execution Phase)**:
   - Sampling fills a buffer.
   - FFT computes dominant frequency.
   - Aggregation calculates statistics.
   - MQTT publishes results.
   - Performance publishes metrics.
   - Tasks are synchronized via semaphores/queues.

3. **Sleep (Shutdown Phase)**:
   - Waits for task completion (12-second timeout).
   - Increments `cycle_count` and sleeps for 2.5 seconds.
   - Preserves state in RTC memory.


## Data Flow
Data moves through tasks in a pipeline:

1. **Sampling Task**:
   - Signal: `2.0 * sin(2π * 150 * t) + 4.0 * sin(2π * 100 * t) + 1.0`.
   - Writes to active buffer, swaps buffers, signals FFT (`xSamplingCompleteSemaphore`).
   - **Output**: 256-sample buffer.

2. **FFT Task**:
   - Reads inactive buffer, applies FFT (Hamming window).
   - Computes `max_frequency`, sets `new_sample_rate = 2.2 * max_frequency`.
   - Sends `FFTResult_t` to `xFFTResultQueue`.
   - **Output**: `{ max_frequency, max_magnitude, new_sample_rate }`.

3. **Aggregate Task**:
   - Receives `FFTResult_t`, reads buffer.
   - Computes `mean`, `median`, `mse` over 7 seconds.
   - Sends `AggregateResult_t` to `xAggregateQueue`.
   - **Output**: `{ mean, median, mse }`.

4. **MQTT Task**:
   - Receives `AggregateResult_t`, stores in `aggregate_result`.
   - Signals Performance Task (`xMQTTCompleteSemaphore`).
   - Tracks latency.
   - **Output**: Stored metrics, latency.

5. **Performance Task**:
   - Retrieves `aggregate_result`, collects metrics.
   - Publishes JSON:
     ```json
     {
       "mean": <float>,
       "median": <float>,
       "mse": <float>,
       "cycle": <int>,
       "latency_ms": <float>,
       "rate": <int>,
       "mqtt_bytes": <int>,
       "heap": <int>
     }
     ```
   - **Output**: MQTT message.

**Synchronization**:
- Mutexes prevent conflicts.
- Queues enable asynchronous data transfer.
- Semaphores enforce task order.

## Design Choices and Rationale
Key decisions enhance efficiency and reliability:

1. **FreeRTOS and Task Prioritization**:
   - **Why**: Ensures real-time sampling isn’t delayed.
   - **How**: Sampling (priority 5) on core 0, FFT/Aggregate (3) on core 1.
   - **Example 1**: Core separation balances load, e.g., Sampling runs uninterrupted while FFT processes.

2. **Double-Buffering**:
   - **Why**: Enables concurrent sampling and processing.
   - **How**: Two buffers alternate, guarded by `xSampleBufferMutex`.
   - **Example 2**: Sampling writes to `sample_buffer_1` while FFT reads `sample_buffer_2`.

3. **Dynamic Sampling Rate**:
   - **Why**: Saves power by matching rate to signal needs.
   - **How**: Rate set to `2.2 * max_frequency` (10–32,000 Hz).
   - **Example 3**: 150 Hz signal yields ~330 Hz rate, reducing CPU usage vs. 32 kHz.

4. **Deep Sleep**:
   - **Why**: Extends battery life.
   - **How**: 2.5-second sleep, RTC memory for state.
   - **Example**: ~40% duty cycle (2 seconds active, 2.5 seconds asleep).

5. **Watchdog Timer**:
   - **Why**: Recovers from hangs.
   - **How**: 15-second timeout.
   - **Example**: Resets if FFT stalls.

6. **MQTT**:
   - **Why**: Lightweight IoT protocol.
   - **How**: Publishes to `test.mosquitto.org`.
   - **Example**: JSON enables easy parsing.

## Configuration Parameters
| Constant                  | Value        | Description                                      |
|---------------------------|--------------|--------------------------------------------------|
| `BUFFER_SIZE`             | 256          | Samples per buffer                               |
| `WINDOW_SEC`              | 7            | Aggregation window (seconds)                     |
| `MIN_SAMPLE_RATE`         | 10           | Min sampling rate (Hz)                           |
| `MAX_SAMPLE_RATE`         | 32000        | Max sampling rate (Hz)                           |
| `SLEEP_DURATION_US`       | 2500000      | Sleep duration (µs, 2.5s)                        |
| `WIFI_RECONNECT_ATTEMPTS` | 10           | Max WiFi retries                                 |
| `MQTT_RECONNECT_ATTEMPTS` | 5            | Max MQTT retries                                 |
| `TASK_PRIORITY_*`         | 2–5          | Task priorities                                  |
| `STACK_SIZE_*`            | 4096–8192    | Task stack sizes (bytes)                         |
| `WIFI_SSID`               | "Najeh's S25 Ultra" | WiFi SSID                                 |
| `WIFI_PASSWORD`           | "white1xx"   | WiFi password                                    |
| `MQTT_SERVER`             | "test.mosquitto.org" | MQTT broker                              |
| `MQTT_PORT`               | 1883         | MQTT port                                        |
| `MQTT_TOPIC_METRICS`      | "whitex/sensor/metrics" | MQTT topic                            |

**Rationale**:
- `BUFFER_SIZE = 256`: FFT-friendly, memory-efficient.
- `WINDOW_SEC = 7`: Balances statistics and computation.
- `SLEEP_DURATION_US = 2500000`: Frequent updates, low power.

## Dependencies
- **FreeRTOS**: Task management (ESP32 Arduino core).
- **WiFi**: ESP32 WiFi (Arduino core).
- **PubSubClient**: MQTT
- **ArduinoFFT**: FFT 
- **ArduinoJson**: JSON
- **ESP32 Core**: ADC, timers, sleep, WDT 

Install via Arduino IDE or PlatformIO.

## Error Handling and Reliability
- **Memory Checks**: Validates allocations, restarts on failure.
- **Timeouts**: Task waits (e.g., 10 seconds for FFT) prevent deadlocks.
- **Watchdog**: 15-second reset.
- **Reconnections**: WiFi (10 retries), MQTT (5 retries).
- **RTC Memory**: Preserves state.
- **Degradation**: Continues processing if WiFi/MQTT fails.

**Example**: `vAggregateTask` logs queue timeouts and proceeds to sleep.

## Grafana Visualization
Grafana enables real-time visualization of aggregated metrics (`mean`, `median`, `mse`, `latency_ms`, `rate`, `heap`) as time-series charts, providing insights into signal behavior and system performance.

- **Purpose**: To monitor trends, e.g., signal stability (`mean` ~1.0, `mse` ~4.0) or processing delays (`latency_ms`).
- **Setup**: Uses an MQTT data source to fetch JSON payloads from `whitex/sensor/metrics`.
- **Chart Types**: Table for multiple metrics.
- **Example Dashboard**:
  - **Mean**: Tracks signal average, expected ~1.0 due to offset.
  - **MSE**: Shows signal variance, ~4.0 for synthetic signal.
  - **Latency**: Monitors processing time, typically 1000–2000 ms.
  - **Rate**: Displays sampling rate, e.g., ~330 Hz after FFT adjustment.
  - See screenshot:
    ![Grafana Chart](docs/grafana_chart.png)
    *Figure: Grafana dashboard showing time-series plots of mean, MSE, latency, and sampling rate.*

Detailed setup instructions are in the [Hands-On Walkthrough](#hands-on-walkthrough).

## Hands-On Walkthrough
This section guides you through setting up the ESP32 application, visualizing data in Grafana, and preparing for LoRaWAN with TTN.

### Prerequisites
- **Hardware**:
  - ESP32 board (e.g., ESP32-DevKitC).
  - USB cable.
  - (For LoRaWAN) LoRa-capable board (e.g., Heltec WiFi LoRa 32) or external module (e.g., SX1276).
  - (Optional) Computer for MQTT/Grafana.
- **Software**:
  - Arduino IDE or PlatformIO.
  - ESP32 board support (`espressif/arduino-esp32`, `v2.0.9+`).
  - Libraries: `PubSubClient`, `ArduinoFFT`, `ArduinoJson`.
  - (For Grafana) Docker or Grafana Cloud, MQTT broker (e.g., Mosquitto).
  - (For LoRaWAN) TTN account.
  - Serial monitor (Arduino IDE, PuTTY).
  - MQTT client (e.g., MQTT Explorer).

### Step-by-Step Setup
1. **Install Arduino IDE**:
   - Download from [arduino.cc](https://www.arduino.cc/en/software).
   - Install and open.

2. **Add ESP32 Support**:
   - `File > Preferences`.
   - Add to `Additional Boards Manager URLs`:  
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - `Tools > Board > Boards Manager`, install `esp32`.

3. **Install Libraries**:
   - `Sketch > Include Library > Manage Libraries`.
   - Install:
     - `PubSubClient` (Nick O’Leary).
     - `ArduinoFFT` (Enrique Condes).
     - `ArduinoJson` (Benoit Blanchon).
   - For PlatformIO, edit `platformio.ini`:
     ```ini
     lib_deps =
       knolleary/PubSubClient@^2.8
       kosme/ArduinoFFT@^2.0.2
       bblanchon/ArduinoJson@^6.21.0
     ```

4. **Configure the Code**:
   - Copy code to `esp32_sensor.ino`.
   - Update WiFi:
     ```cpp
     const char* WIFI_SSID = "Your_SSID";
     const char* WIFI_PASSWORD = "Your_Password";
     ```
   - (Optional) MQTT settings:
     ```cpp
     const char* MQTT_SERVER = "your.broker.com";
     const int MQTT_PORT = 1883;
     const char* MQTT_TOPIC_METRICS = "your/topic";
     ```

5. **Connect ESP32**:
   - Connect via USB.
   - `Tools > Board > ESP32 Arduino > ESP32 Dev Module`.
   - Select port (`Tools > Port`, e.g., `COM3`).

6. **Upload Code**:
   - `Sketch > Upload`.
   - Resolve errors (check libraries, board, port).
   - Upload: ~10–20 seconds.

7. **Monitor Serial**:
   - `Tools > Serial Monitor`, 115200 baud.
   - Output:
     ```
     --- Boot Count: 1 ---
     Cycle Count: 0
     Current Sample Rate: 32000 Hz
     SamplingTask started at ...
     FFTTask: Max freq = 150.00 Hz, Mag = ...
     PerformanceTask: Publishing JSON: {"mean":...,"median":...,...}
     Cycle 1 finished. Sleeping for 2.50 seconds...
     ```
   - Cycles every ~4.5 seconds (2 seconds active, 2.5 seconds sleep).

8. **Verify MQTT**:
   - Use MQTT Explorer ([mqttexplorer.com](http://mqttexplorer.com)).
   - Connect to `test.mosquitto.org:1883`, no credentials.
   - Subscribe to `whitex/sensor/metrics`.
   - Expect JSON:
     ```json
     {
       "mean": 1.0,
       "median": 1.0,
       "mse": 4.0,
       "cycle": 1,
       "latency_ms": 1500.0,
       "rate": 330,
       "mqtt_bytes": 256,
       "heap": 180000
     }
     ```

9. **Set Up Grafana**:
   - **Install Grafana**:
     - Docker:
       ```bash
       docker run -d -p 3000:3000 --name grafana grafana/grafana:latest
       ```
     - Or use Grafana Cloud ([grafana.com](https://grafana.com)).
     - Access: `http://localhost:3000`, login (`admin/admin`).
   - **Install MQTT Plugin**:
     - `Configuration > Plugins`, search `MQTT`, install `Grafana MQTT Data Source`.
     - Or:
       ```bash
       docker exec grafana grafana-cli plugins install grafana-mqtt-datasource
       ```
   - **Configure Data Source**:
     - `Configuration > Data Sources > Add data source`, select `MQTT`.
     - URL: `mqtt://test.mosquitto.org:1883`.
     - Topic: `whitex/sensor/metrics`.
     - No authentication.
     - Save and test.
   - **Create Dashboard**:
     - `Create > Dashboard > Add new panel`.
     - Query:
       - Data source: MQTT.
       - Topic: `whitex/sensor/metrics`.
       - Fields: `mean`, `median`, `mse`, `latency_ms`, `rate`, `heap`.
       - Format: Time series.
     - Visualization: Line chart.
     - Metrics:
       - `mean`: ~1.0.
       - `mse`: ~4.0.
       - `latency_ms`: 1000–2000 ms.
     - Time range: Last 1 hour.
     - Save.
   - **View**:
     - Updates every ~4.5 seconds.
     - Trends: Stable `mean`, varying `latency_ms`.
     - See [Grafana Visualization](#grafana-visualization) for screenshot.

### Setting Up LoRaWAN with TTN
The current code uses WiFi/MQTT, but you can register a device in The Things Network (TTN) for future LoRaWAN integration. This requires a LoRa-capable board (e.g., Heltec WiFi LoRa 32, TTGO LoRa32) or an external module (e.g., SX1276). Below is how to set up the device in TTN with necessary parameters.

1. **Create TTN Account**:
   - Sign up at [thethingsnetwork.org](https://www.thethingsnetwork.org).
   - Log in to TTN Console.

2. **Verify Gateway**:
   - Ensure a TTN gateway is nearby ([ttnmapper.org](https://ttnmapper.org)).
   - Or register your own (e.g., RAKwireless) in TTN Console.

3. **Add Application**:
   - `Applications > Add application`.
   - Settings:
     - ID: `esp32-sensor-app`.
     - Name: `ESP32 Sensor App`.
   - Save.

4. **Add Device**:
   - In `esp32-sensor-app`, `Add end device`.
   - Manual registration:
     - **Parameters**:
       - **DevEUI**: Unique 8-byte device identifier (generate in TTN or use board-specific).
       - **AppEUI**: 8-byte application identifier (generate in TTN).
       - **AppKey**: 16-byte encryption key (generate in TTN).
     - Frequency plan: Match region (e.g., EU868, US915).
     - LoRaWAN version: 1.0.2.
   - Save.
   - Screenshot:
     ![TTN Setup](docs/ttn_setup.png)
     *Figure: TTN Console showing device registration with DevEUI, AppEUI, and AppKey.*

### Troubleshooting
- **Serial**: Check 115200 baud, port.
- **WiFi**: Verify SSID/password.
- **MQTT**: Ping broker, check topic.
- **Grafana**: Confirm MQTT source, topic.
- **TTN**: Ensure gateway, correct keys.

