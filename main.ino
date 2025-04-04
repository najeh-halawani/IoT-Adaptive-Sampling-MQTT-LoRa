#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include <arduinoFFT.h>
#include <esp_sleep.h>

// WiFi and MQTT Configurations
const char* ssid = "whitex";
const char* password = "whitewhite";
const char* mqtt_server = "test.mosquitto.org"; // Use local MQTT broker instead of public one
const int mqtt_port = 1883;

// Pin definitions and sensor parameters
#define ADC_PIN ADC1_CHANNEL_6 // GPIO34, check ESP32 datasheet for valid ADC pins
#define ADC_UNIT ADC_UNIT_1

// Queue and timing structures
QueueHandle_t sensorQueue;        // Queue for passing sensor data between tasks
QueueHandle_t timestampQueue;     // Queue for passing timestamps
#define QUEUE_SIZE 100            // Increased queue capacity
#define WINDOW_SIZE_SECONDS 5     // 5-second window as per assignment
#define SAMPLES 128               // Increased FFT samples for better resolution
#define MIN_SAMPLING_FREQUENCY 10 // Minimum sampling frequency (Hz)

// Define as variable, not constant, so it can be updated
int MAX_SAMPLING_FREQUENCY = 1000; // Initial max value, will be measured in setup()

// Signal parameters for testing - simulating a_k*sin(f_k)
const double amplitude1 = 2.0;   // a_1
const double frequency1 = 3.0;   // f_1 (Hz)
const double amplitude2 = 4.0;   // a_2
const double frequency2 = 5.0;   // f_2 (Hz)

// FFT setup
double vReal[SAMPLES];
double vImag[SAMPLES] = {0};

// Sampling and timing variables
int samplingFrequency = 100;     // Initial sampling frequency (Hz)
int optimalSamplingFrequency = samplingFrequency;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;
unsigned long totalSamplesCollected = 0;
unsigned long dataBytesTransmitted = 0;

// Energy and performance metrics
double originalEnergyConsumption = 0;  // Using sample count as proxy for energy
double adaptiveEnergyConsumption = 0;
unsigned long originalDataVolume = 0;
unsigned long adaptiveDataVolume = 0;
unsigned long latencyMeasurements = 0;
unsigned long totalLatency = 0;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);

WiFiClient espClient;
PubSubClient client(espClient);

// Function Prototypes
void setupWiFi();
void setupMQTT();
void setupADC();
void sensorTask(void* pvParameters);
void simulateSensorTask(void* pvParameters); // Added for signal simulation
void averageTask(void* pvParameters);
void transmitToEdgeServer(double average, unsigned long timestamp);
void fftTask(void* pvParameters);
void adjustSamplingFrequency(double dominantFrequency);
void checkWiFiConnection();
void checkMQTTConnection();
void metricsTask(void* pvParameters);
double generateSignal(unsigned long timeMs);

void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize
    
    Serial.println("\n\n--- ESP32 Adaptive Sampling IoT System Starting ---");
    
    // Setup WiFi, MQTT, and ADC
    setupWiFi();
    setupMQTT();
    setupADC();
    
    // Create queues
    sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(double));
    timestampQueue = xQueueCreate(QUEUE_SIZE, sizeof(unsigned long));
    
    if (sensorQueue == NULL || timestampQueue == NULL) {
        Serial.println("Error creating queues");
        while (true); // Block execution
    }
    
    // Measure maximum sampling frequency capability
    unsigned long startTime = millis();
    int sampleCount = 0;
    while (millis() - startTime < 1000) { // Test for 1 second
        adc1_get_raw(ADC_PIN);
        sampleCount++;
    }
    
    // Calculate and set max frequency (now works as MAX_SAMPLING_FREQUENCY is a variable)
    int measuredMaxFreq = sampleCount;
    MAX_SAMPLING_FREQUENCY = (measuredMaxFreq > 1000) ? 1000 : measuredMaxFreq; // Cap at 1kHz
    
    Serial.print("Measured maximum sampling frequency: ");
    Serial.print(measuredMaxFreq);
    Serial.println(" Hz");
    
    // Create tasks
    xTaskCreate(simulateSensorTask, "Simulated Sensor", 4096, NULL, 3, NULL);
    xTaskCreate(averageTask, "Average Calculator", 4096, NULL, 2, NULL);
    xTaskCreate(fftTask, "FFT Analyzer", 8192, NULL, 1, NULL); // Higher stack as FFT needs more memory
    xTaskCreate(metricsTask, "Metrics Reporter", 4096, NULL, 1, NULL);
    
    Serial.println("All tasks created successfully");
}

void loop() {
    // Main loop handles connection maintenance
    checkWiFiConnection();
    checkMQTTConnection();
    client.loop();
    delay(100); // Give some time for other tasks
}

void setupADC() {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12); // 12-bit resolution
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_11); // 0-3.3V range
    
    Serial.println("ADC configured successfully");
}

// Function to simulate sensor data with the required signal form
double generateSignal(unsigned long timeMs) {
    double t = timeMs / 1000.0; // Convert to seconds
    
    // Generate sum of sines: a_1*sin(2π*f_1*t) + a_2*sin(2π*f_2*t)
    double signal = amplitude1 * sin(2 * PI * frequency1 * t) + 
                   amplitude2 * sin(2 * PI * frequency2 * t);
    
    // Add some noise to make it more realistic
    signal += (random(100) - 50) / 100.0;
    
    // Map to ADC range (0-4095 for 12-bit resolution)
    return map(signal * 100, -600, 600, 0, 4095);
}

// Task to simulate sensor readings with the specified signal
void simulateSensorTask(void* pvParameters) {
    double sensorValue;
    unsigned long timestamp;
    TickType_t delayPeriod;
    
    while (true) {
        // Calculate delay based on current sampling frequency
        int currentSamplingFreq = max(samplingFrequency, MIN_SAMPLING_FREQUENCY);
        delayPeriod = pdMS_TO_TICKS(1000 / currentSamplingFreq);
        
        // Generate timestamp and simulated sensor value
        timestamp = millis();
        sensorValue = generateSignal(timestamp);
        
        // Send data and timestamp to queues
        if (xQueueSend(sensorQueue, &sensorValue, 0) != pdPASS) {
            Serial.println("Sensor queue full");
        }
        if (xQueueSend(timestampQueue, &timestamp, 0) != pdPASS) {
            Serial.println("Timestamp queue full");
        }
        
        // Track metrics
        totalSamplesCollected++;
        if (samplingFrequency == MAX_SAMPLING_FREQUENCY) {
            originalEnergyConsumption++;
        } else {
            adaptiveEnergyConsumption++;
        }
        
        // Delay until next sample
        vTaskDelay(delayPeriod);
    }
}

// Real sensor task - not used in this implementation but kept for reference
void sensorTask(void* pvParameters) {
    double sensorValue;
    unsigned long timestamp;
    TickType_t delayPeriod;
    
    while (true) {
        // Calculate delay based on current sampling frequency
        int currentSamplingFreq = max(samplingFrequency, MIN_SAMPLING_FREQUENCY);
        delayPeriod = pdMS_TO_TICKS(1000 / currentSamplingFreq);
        
        // Read actual sensor value and record timestamp
        timestamp = millis();
        sensorValue = adc1_get_raw(ADC_PIN);
        
        // Send data and timestamp to queues
        if (xQueueSend(sensorQueue, &sensorValue, 0) != pdPASS) {
            Serial.println("Sensor queue full");
        }
        if (xQueueSend(timestampQueue, &timestamp, 0) != pdPASS) {
            Serial.println("Timestamp queue full");
        }
        
        // Track metrics
        totalSamplesCollected++;
        if (samplingFrequency == MAX_SAMPLING_FREQUENCY) {
            originalEnergyConsumption++;
        } else {
            adaptiveEnergyConsumption++;
        }
        
        // Delay until next sample
        vTaskDelay(delayPeriod);
    }
}

// FFT task to analyze the signal frequency and adjust sampling rate
void fftTask(void* pvParameters) {
    // Wait for some samples to be collected first
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (true) {
        // Collect SAMPLES data points for FFT
        Serial.println("Starting FFT analysis...");
        
        // Reset FFT arrays
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = 0;
            vImag[i] = 0;
        }
        
        // Collect samples for FFT (using simulated or actual data)
        int currentSamplingFreq = max(samplingFrequency, MIN_SAMPLING_FREQUENCY);
        unsigned long startSample = millis();
        
        for (int i = 0; i < SAMPLES; i++) {
            // Direct sampling for FFT analysis
            vReal[i] = generateSignal(millis());
            
            // Delay according to current sampling frequency
            vTaskDelay(pdMS_TO_TICKS(1000 / currentSamplingFreq));
        }
        
        // Update FFT configuration with current sampling frequency
        FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, currentSamplingFreq);
        
        // Perform FFT
        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);
        
        // Find dominant frequency
        double dominantFrequency = FFT.majorPeak(vReal, SAMPLES, currentSamplingFreq);
        
        Serial.print("FFT Analysis - Dominant Frequency: ");
        Serial.print(dominantFrequency, 2);
        Serial.println(" Hz");
        
        // Adjust sampling frequency based on dominant frequency
        adjustSamplingFrequency(dominantFrequency);
        
        // Run FFT every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Function to adjust sampling frequency based on signal characteristics
void adjustSamplingFrequency(double dominantFrequency) {
    // Only adjust if a valid frequency is detected
    if (dominantFrequency > 0) {
        // Apply Nyquist theorem (2x highest frequency) with safety margin
        optimalSamplingFrequency = ceil(2.2 * dominantFrequency);
        
        // Ensure frequency stays within bounds - fix the type mismatch
        optimalSamplingFrequency = max(optimalSamplingFrequency, (int)MIN_SAMPLING_FREQUENCY);
        optimalSamplingFrequency = min(optimalSamplingFrequency, MAX_SAMPLING_FREQUENCY);
        
        // Apply the new sampling frequency
        samplingFrequency = optimalSamplingFrequency;
        
        Serial.print("Sampling frequency adjusted to: ");
        Serial.print(samplingFrequency);
        Serial.println(" Hz");
    } else {
        Serial.println("No dominant frequency detected. Keeping current sampling rate.");
    }
}

// Average Task to compute window averages and transmit to edge server
void averageTask(void* pvParameters) {
    double sensorValue;
    unsigned long timestamp;
    double sum = 0;
    int count = 0;
    
    // Circular buffer for windowed data
    typedef struct {
        double value;
        unsigned long timestamp;
    } SensorReading;
    
    SensorReading* window = (SensorReading*)pvPortMalloc(QUEUE_SIZE * sizeof(SensorReading));
    if (window == NULL) {
        Serial.println("Failed to allocate memory for window buffer");
        vTaskDelete(NULL);
        return;
    }
    
    int windowStart = 0;
    int windowEnd = 0;
    unsigned long windowStartTime = 0;
    
    // Wait for initial samples
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (true) {
        // Check if there's data in the queue
        if (xQueueReceive(sensorQueue, &sensorValue, 0) == pdPASS &&
            xQueueReceive(timestampQueue, &timestamp, 0) == pdPASS) {
                
            // First reading establishes window start time
            if (count == 0) {
                windowStartTime = timestamp;
            }
            
            // Add new reading to window
            window[windowEnd].value = sensorValue;
            window[windowEnd].timestamp = timestamp;
            windowEnd = (windowEnd + 1) % QUEUE_SIZE;
            count++;
            
            // Update sum
            sum += sensorValue;
            
            // Remove old values that are outside the time window
            while (windowStart != windowEnd && 
                  (timestamp - window[windowStart].timestamp) > (WINDOW_SIZE_SECONDS * 1000)) {
                sum -= window[windowStart].value;
                windowStart = (windowStart + 1) % QUEUE_SIZE;
                count--;
            }
            
            // If we have a full window, transmit the average
            if ((timestamp - windowStartTime) >= (WINDOW_SIZE_SECONDS * 1000)) {
                double average = sum / count;
                
                Serial.print("Window Average: ");
                Serial.print(average, 2);
                Serial.print(" (");
                Serial.print(count);
                Serial.println(" samples)");
                
                // Transmit to edge server
                transmitToEdgeServer(average, timestamp);
                
                // Reset for next window
                windowStartTime = timestamp;
            }
        }
        
        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Function to transmit data to the edge server via MQTT
void transmitToEdgeServer(double average, unsigned long timestamp) {
    if (!client.connected()) {
        Serial.println("MQTT disconnected. Attempting reconnection...");
        setupMQTT();
        if (!client.connected()) {
            Serial.println("Failed to reconnect MQTT. Data not transmitted.");
            return;
        }
    }
    
    // Create JSON message with average, timestamp, and sampling frequency info
    char msg[128];
    sprintf(msg, "{\"average\":%.2f,\"timestamp\":%lu,\"sampling_freq\":%d,\"window_size\":%d}", 
           average, timestamp, samplingFrequency, WINDOW_SIZE_SECONDS);
    
    // Record start time for latency calculation
    unsigned long startTime = millis();
    
    // Publish to MQTT topic
    if (client.publish("sensor/average", msg)) {
        // Message was published successfully
        unsigned long endTime = millis();
        unsigned long latency = endTime - startTime;
        
        // Update metrics
        dataBytesTransmitted += strlen(msg);
        latencyMeasurements++;
        totalLatency += latency;
        
        // Log message and latency
        Serial.print("Published to Edge Server: ");
        Serial.println(msg);
        Serial.print("Transmission latency: ");
        Serial.print(latency);
        Serial.println(" ms");
        
        // Track data volume metrics
        if (samplingFrequency == MAX_SAMPLING_FREQUENCY) {
            originalDataVolume += strlen(msg);
        } else {
            adaptiveDataVolume += strlen(msg);
        }
    } else {
        Serial.println("Failed to publish message");
    }
}

// Task to report system metrics periodically
void metricsTask(void* pvParameters) {
    // Wait for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(30000));
    
    while (true) {
        // Calculate metrics
        double energySavingPercentage = 0;
        if (originalEnergyConsumption > 0) {
            energySavingPercentage = 100.0 * (1.0 - (double)adaptiveEnergyConsumption / originalEnergyConsumption);
        }
        
        double dataReductionPercentage = 0;
        if (originalDataVolume > 0) {
            dataReductionPercentage = 100.0 * (1.0 - (double)adaptiveDataVolume / originalDataVolume);
        }
        
        double avgLatency = 0;
        if (latencyMeasurements > 0) {
            avgLatency = (double)totalLatency / latencyMeasurements;
        }
        
        // Print metrics report
        Serial.println("\n--- System Metrics Report ---");
        Serial.print("Current Sampling Frequency: ");
        Serial.print(samplingFrequency);
        Serial.println(" Hz");
        
        Serial.print("Energy Savings: ");
        Serial.print(energySavingPercentage, 2);
        Serial.println("%");
        
        Serial.print("Data Volume Reduction: ");
        Serial.print(dataReductionPercentage, 2);
        Serial.println("%");
        
        Serial.print("Average End-to-End Latency: ");
        Serial.print(avgLatency, 2);
        Serial.println(" ms");
        
        Serial.print("Total Samples Collected: ");
        Serial.println(totalSamplesCollected);
        
        Serial.print("Total Data Transmitted: ");
        Serial.print(dataBytesTransmitted);
        Serial.println(" bytes");
        Serial.println("------------------------\n");
        
        // Send metrics to server
        if (client.connected()) {
            char metricsMsg[256];
            sprintf(metricsMsg, 
                  "{\"metrics\":{\"energy_savings\":%.2f,\"data_reduction\":%.2f,\"avg_latency\":%.2f,\"sampling_freq\":%d}}",
                  energySavingPercentage, dataReductionPercentage, avgLatency, samplingFrequency);
            
            client.publish("sensor/metrics", metricsMsg);
        }
        
        // Report every minute
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// Function to connect to WiFi
void setupWiFi() {
    Serial.println("\nSetting up WiFi connection...");
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected successfully");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi connection failed");
    }
}

// Function to connect to MQTT broker
void setupMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    
    // Generate unique client ID
    String clientId = "ESP32-";
    clientId += String(random(0xffff), HEX);
    
    Serial.print("Connecting to MQTT broker as ");
    Serial.println(clientId);
    
    int retries = 0;
    while (!client.connected() && retries < 5) {
        if (client.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("Failed to connect to MQTT, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 2 seconds");
            delay(2000);
            retries++;
        }
    }
}

void checkWiFiConnection() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWifiCheck > 30000) {
        lastWifiCheck = currentMillis;
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi connection lost. Reconnecting...");
            WiFi.disconnect();
            setupWiFi();
        }
    }
}

void checkMQTTConnection() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastMqttCheck > 10000) {
        lastMqttCheck = currentMillis;
        if (!client.connected()) {
            setupMQTT();
        }
    }
}