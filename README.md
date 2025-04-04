# Adaptive Sampling on Heltec ESP32 V3 with MQTT and LoRaWAN

## Table of Contents

- [Adaptive Sampling on Heltec ESP32 V3 with MQTT and LoRaWAN](#adaptive-sampling-on-heltec-esp32-v3-with-mqtt-and-lorawan)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Features](#features)
  - [Hardware Requirements](#hardware-requirements)
  - [Software Dependencies](#software-dependencies)
  - [Configuration](#configuration)
    - [WiFi Configuration](#wifi-configuration)
    - [MQTT Configuration](#mqtt-configuration)
    - [LoRaWAN Configuration](#lorawan-configuration)
    - [Sampling Parameters](#sampling-parameters)
    - [Aggregation Parameters](#aggregation-parameters)
    - [Performance Measurement](#performance-measurement)
  - [Data Structures](#data-structures)
  - [Task Breakdown](#task-breakdown)
    - [WiFi and MQTT](#wifi-and-mqtt)
    - [Sampling and Signal Processing](#sampling-and-signal-processing)
    - [LoRa Communication](#lora-communication)
    - [Performance Monitoring](#performance-monitoring)
  - [LoRaWAN Events](#lorawan-events)
  - [How to Build and Flash](#how-to-build-and-flash)

---

## Introduction

This firmware enables an ESP32 adaptive sampling system that collects sensor data, processes it using FFT, and transmits relevant metrics over MQTT and LoRaWAN. It dynamically adjusts the sampling rate based on signal characteristics to optimize data transmission.

## Features

- Adaptive sampling based on signal analysis
- Real-time FFT analysis using ArduinoFFT
- Data aggregation and transmission over MQTT and LoRaWAN
- Performance monitoring with energy savings calculations
- Multi-tasking using FreeRTOS

## Hardware Requirements

- ESP32 development board (e.g., Heltec ESP32 V3)
- LoRaWAN module
- WiFi network access
- Sensor for signal input

## Software Dependencies

This firmware relies on the following libraries:

- `WiFi.h` for network connectivity
- `PubSubClient.h` for MQTT communication
- `arduinoFFT.h` for signal processing
- `Preferences.h` for persistent storage
- `lmic.h` for LoRaWAN communication
- `LoRaWan_APP.h` for LoRaWAN communication

## Configuration

### WiFi Configuration

Modify the following constants in the code to set up WiFi connectivity:
```cpp
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";
```

### MQTT Configuration

Define the MQTT broker, port, and topics:
```cpp
const char* mqtt_server = "test.mosquitto.org";
const int mqtt_port = 1883;
const char* mqtt_topic_average = "white/sensor/average";
const char* mqtt_topic_metrics = "white/sensor/metrics";
const char* mqtt_topic_fft_analysis = "white/sensor/fft_analysis";
```

### LoRaWAN Configuration
![TTN](https://github.com/najeh-halawani/IoT-Adaptive-Sampling-MQTT-LoRa/blob/main/esp32-ttn.png)

Update the following parameters for LoRaWAN connectivity:
```cpp
static const u1_t PROGMEM APPEUI[8] = { /* LoRaWAN App EUI */ }; // LSB
static const u1_t PROGMEM DEVEUI[8] = { /* LoRaWAN Device EUI */ }; // LSB
static const u1_t PROGMEM APPKEY[16] = { /* LoRaWAN Application Key */ }; // MSB
```

### Sampling Parameters

The sampling rate is dynamically adjusted based on signal properties. The key parameters are:
```cpp
const double MAX_SAMPLING_FREQ_HZ = 44000.0;
const double MIN_SAMPLING_FREQ_HZ = 10.0;
const uint16_t FFT_WINDOW_SIZE = 1024;
const double NYQUIST_FACTOR = 2.2;
```

### Aggregation Parameters

Data is aggregated over a defined window before transmission:
```cpp
const unsigned long AGGREGATION_WINDOW_MS = 5000;  // 5 seconds
```

### Performance Measurement

Performance monitoring variables track transmission efficiency:
```cpp
const unsigned long EXPERIMENT_DURATION_MS = 60000;  // 1-minute experiment
const bool ENABLE_ADAPTIVE_SAMPLING = true;
```

## Data Structures

```cpp
typedef struct {
  unsigned long timestamp_us;
  float value;
} SampleData_t;

typedef struct {
  unsigned long window_start_us;
  float average_value;
  double f_max;
  double fs_actual;
} AggregateData_t;

typedef struct {
  unsigned long duration_ms;
  double avg_sampling_freq;
  unsigned long bytes_sent;
  unsigned long avg_latency_us;
  float energy_saved_percent;
  float data_volume_reduction_percent;
} PerformanceMetrics_t;
```

## Task Breakdown

### WiFi and MQTT
- `connectWiFi()`: Establishes WiFi connection.
- `reconnectMQTT()`: Reconnects to the MQTT broker if disconnected.
- `mqttCommTask()`: Handles MQTT data transmission.

### Sampling and Signal Processing
- `samplingTask()`: Collects sensor data at an adaptive rate.
- `analysisTask()`: Processes collected data using FFT.
- `aggregationTask()`: Aggregates processed data for transmission.

### LoRa Communication
- `setupLoRaWAN()`: Initializes LoRaWAN communication.
- `loraCommTask()`: Manages LoRaWAN data transmission.
- `do_send()`: Sends aggregated data via LoRa.

### Performance Monitoring
- `performanceMonitorTask()`: Tracks data efficiency and resource usage.

## LoRaWAN Events
The firmware handles multiple LoRaWAN events, including:

- `EV_JOINING`: Joining the network
- `EV_JOINED`: Successfully joined
- `EV_TXCOMPLETE`: Transmission completed
- `EV_LINK_DEAD`: Lost connection

## How to Build and Flash

1. Install dependencies via Arduino Library Manager.
2. Configure WiFi, MQTT, and LoRaWAN settings.
3. Connect ESP32 to your PC via USB.
4. Use Arduino IDE or PlatformIO to compile and upload the firmware.
5. Open Serial Monitor to debug and check logs.



# Grafana 
![Grafana Dashboard](https://github.com/najeh-halawani/IoT-Adaptive-Sampling-MQTT-LoRa/blob/main/grafana-dashboard.png)


# Metrics 
![Metrics](https://github.com/najeh-halawani/IoT-Adaptive-Sampling-MQTT-LoRa/blob/main/metrics.png)
