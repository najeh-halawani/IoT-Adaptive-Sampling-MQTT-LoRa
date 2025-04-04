#include <WiFi.h>
#include <PubSubClient.h>
#include <driver/adc.h>
#include <arduinoFFT.h>  

// WiFi and MQTT Configurations
const char* ssid = "whitex";
const char* password = "whitewhite";
const char* mqtt_server = "test.mosquitto.org";


#define Analog_PIC 19  // Analog pin for measures
QueueHandle_t sensorQueue;  // Queue for passing sensor data between tasks
#define QUEUE_SIZE 10        // Queue capacity
#define WINDOW_SIZE 5        // Size of the moving average window
#define SAMPLES 64           // FFT samples
#define MIN_SAMPLING_FREQUENCY 10  // Minimum sampling frequency (Hz) to prevent division by zero

// FFT setup
double vReal[SAMPLES];
double vImag[SAMPLES] = {0};  // Imaginary part always 0 for real signals

int samplingFrequency = 500;  // Initial sampling frequency (Hz)
int optimalSamplingFrequency = samplingFrequency;
unsigned long lastWifiCheck = 0;
unsigned long lastMqttCheck = 0;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, samplingFrequency);

WiFiClient espClient;
PubSubClient client(espClient);

// Function Prototypes
void setupWiFi();
void setupMQTT();
// void setupLoRa();
void sensorTask(void* pvParameters);
void AverageTask(void* pvParameters);
void transmitToEdgeServer(double average);
// void transmitToCloud(double average);
void fftTask(void* pvParameters);
void adjustSamplingFrequency(double dominantFrequency);
void checkWiFiConnection();
void checkMQTTConnection();


void setup() {
    Serial.begin(115200);

    // Setup WiFi and MQTT
    setupWiFi();
    setupMQTT();

    // Create the queue to handle sensor data
    sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(double));
    if (sensorQueue == NULL) {
        Serial.println("Error creating the queue");
        while (true);  // Block the execution
    }

    // Create the sensor, average, and FFT tasks
    xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 1, NULL);  
    xTaskCreate(AverageTask, "Average Task", 4096, NULL, 1, NULL);  
    xTaskCreate(fftTask, "FFT Task", 4096, NULL, 1, NULL);  
}

void loop() {
    checkWiFiConnection();
    checkMQTTConnection();
    client.loop();  
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
    // Check MQTT connection every 10 seconds
    if (currentMillis - lastMqttCheck > 10000) {
        lastMqttCheck = currentMillis;
        if (!client.connected()) {
            setupMQTT();
        }
    }
}

// Function to connect to WiFi
void setupWiFi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
}

void setupMQTT() {
    client.setServer(mqtt_server, 1883);
    
    // Generate a unique client ID using MAC address
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    Serial.print("Attempting MQTT connection with ID: ");
    Serial.println(clientId);
    
    int retries = 0;
    while (!client.connected() && retries < 5) {
        if (client.connect(clientId.c_str())) {
            Serial.println("MQTT connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" retrying in 5 seconds");
            delay(5000);
            retries++;
        }
    }
}

// Sensor Task to read  data
void sensorTask(void* pvParameters) {
    double sensorValue;
    TickType_t delayPeriod;
    
    while (true) {
        // Calculate delay period based on sampling frequency
        int currentSamplingFreq = (samplingFrequency < MIN_SAMPLING_FREQUENCY) ? 
                               MIN_SAMPLING_FREQUENCY : samplingFrequency;
        delayPeriod = pdMS_TO_TICKS(1000 / currentSamplingFreq);

        // Read photometer sensor value
        sensorValue = analogRead(Analog_PIC);

        // Send the value to the queue
        if (xQueueSend(sensorQueue, &sensorValue, portMAX_DELAY) != pdPASS) {
            Serial.println("Queue full, failed to send data");
        }

        // Delay between sensor readings
        vTaskDelay(delayPeriod);
    }
}

// FFT Task to analyze the dominant frequency and adjust sampling frequency
void fftTask(void* pvParameters) {
    while (true) {
        // Ensure sampling frequency is never zero
        int currentSamplingFreq = (samplingFrequency < MIN_SAMPLING_FREQUENCY) ? 
                               MIN_SAMPLING_FREQUENCY : samplingFrequency;
        
        // Collect SAMPLES data points for FFT
        for (int i = 0; i < SAMPLES; i++) {
            vReal[i] = analogRead(Analog_PIC);  // Sample sensor data
            vTaskDelay(pdMS_TO_TICKS(1000 / currentSamplingFreq));
        }

        FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, currentSamplingFreq);

        // Perform FFT on the collected data
        FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
        FFT.complexToMagnitude(vReal, vImag, SAMPLES);

        // Identify the dominant frequency
        double dominantFrequency = FFT.majorPeak(vReal, SAMPLES, currentSamplingFreq);
        Serial.printf("Dominant Frequency: %.2f Hz\n", dominantFrequency);

        // Adjust sampling frequency based on dominant frequency
        adjustSamplingFrequency(dominantFrequency);

        // Wait before the next FFT calculation
        vTaskDelay(pdMS_TO_TICKS(1000));  // Run FFT every second
    }
}

// Function to adjust the sampling frequency based on the dominant frequency
void adjustSamplingFrequency(double dominantFrequency) {
    if (dominantFrequency > 0) {
        optimalSamplingFrequency = 2.2 * dominantFrequency;  // Nyquist criterion
        Serial.print("Optimal Freq:");
        Serial.println(optimalSamplingFrequency);
        samplingFrequency = (optimalSamplingFrequency < 100) ? optimalSamplingFrequency : 100;  // Cap at 100Hz max
        Serial.printf("Adjusted Sampling Frequency: %d Hz\n", samplingFrequency);
    }
}

// Average Task to compute moving average and transmit results
void AverageTask(void* pvParameters) {
    double sensorValue;
    double distanceReadings[WINDOW_SIZE] = {0};  // Array to store last 5 readings
    int pos = 0;  // Index for circular buffer
    int count = 0;  // Counter to track the number of stored values
    double sum = 0;
    
    while (true) {
        // Receive data from the queue
        if (xQueueReceive(sensorQueue, &sensorValue, portMAX_DELAY)) {
            distanceReadings[pos] = sensorValue;  // Add new reading to circular buffer
            pos = (pos + 1) % WINDOW_SIZE;       // Update buffer position

            // Update the count (ensure it does not exceed WINDOW_SIZE)
            if (count < WINDOW_SIZE) {
                count++;
            }

            // Compute the moving average
            sum = 0;
            for (int i = 0; i < count; i++) {
                sum += distanceReadings[i];
            }
            double averageValue = sum / count;
            Serial.printf("Average Value: %.2f\n", averageValue);

            // Transmit the result to the edge server (MQTT)
            transmitToEdgeServer(averageValue);

            // Transmit the result to the cloud server (LoRa)
            // transmitToCloud(averageValue);
            vTaskDelay(600);
        }
    }
}

// Function to transmit data to the edge server (MQTT over WiFi)
void transmitToEdgeServer(double average) {
    char msg[50];
    sprintf(msg, "Average Value: %f", average);
    client.publish("sensor/average", msg);  // Publish the result over MQTT
    Serial.print("Transmitted to Edge Server: ");
    Serial.println(msg);
}

// Function to transmit data to the cloud server (LoRaWAN)
// void transmitToCloud(double average) {
//     LoRa.beginPacket();
//     LoRa.print(average);
//     LoRa.endPacket();  // Transmit the result over LoRaWAN
//     Serial.print("Transmitted to Cloud: ");
//     Serial.println(average);
// }