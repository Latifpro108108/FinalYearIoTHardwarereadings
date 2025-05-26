#include <Arduino.h>
#include "AZ3166WiFi.h"
#include "SystemWiFi.h"
#include "Wire.h"

// Proxy Server Configuration
const char* proxyHost = "100.100.100.171";  // Your computer's IP address
const int proxyPort = 3000;               // Match the port in your proxy server

// WiFi credentials
char ssid[] = "Dabone's";          // WiFi SSID
char password[] = "12345679";      // WiFi password

// Initialize WiFi client
WiFiClient client;  // Use WiFiClientSecure if needed for HTTPS (requires mbedTLS)

// Buffer for building strings
char buffer[512];
char tempStr[10];
char humStr[10];

// Sensor Constants
#define HTS221_ADDR 0x5F
#define SMOOTH_COUNT 5  // Number of samples for moving average

// Register Map
enum Registers {
  WHO_AM_I_REG = 0x0F,
  CTRL_REG1 = 0x20,
  CALIB_T0_DEGC_X8 = 0x32,
  CALIB_T1_DEGC_X8 = 0x33,
  CALIB_T0_T1_MSB = 0x35,
  CALIB_T0_OUT_L = 0x3C,
  CALIB_T0_OUT_H = 0x3D,
  CALIB_T1_OUT_L = 0x3E,
  CALIB_T1_OUT_H = 0x3F,
  TEMP_OUT_L = 0x2A,
  TEMP_OUT_H = 0x2B,
  CALIB_H0_RH_X2 = 0x30,
  CALIB_H1_RH_X2 = 0x31,
  CALIB_H0_T0_OUT_L = 0x36,
  CALIB_H0_T0_OUT_H = 0x37,
  CALIB_H1_T0_OUT_L = 0x3A,
  CALIB_H1_T0_OUT_H = 0x3B,
  HUMIDITY_OUT_L = 0x28,
  HUMIDITY_OUT_H = 0x29
};

// Calibration Data Structure
struct CalibrationData {
  float T0_degC, T1_degC;
  int16_t T0_out, T1_out;
  float H0_rh, H1_rh;
  int16_t H0_T0_out, H1_T0_out;
};

// Sensor Class
class HTS221 {
private:
  uint8_t address;
  CalibrationData calib;
  float tempBuffer[SMOOTH_COUNT] = {0};
  float humBuffer[SMOOTH_COUNT] = {0};
  uint8_t smoothIndex = 0;

  uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t)1);
    return Wire.read();
  }

  int16_t readInt16(uint8_t regL, uint8_t regH) {
    uint8_t low = readRegister(regL);
    uint8_t high = readRegister(regH);
    return (int16_t)((high << 8) | low);
  }

  float smooth(float *buffer, float newVal) {
    buffer[smoothIndex] = newVal;
    smoothIndex = (smoothIndex + 1) % SMOOTH_COUNT;
    
    float sum = 0;
    for (uint8_t i = 0; i < SMOOTH_COUNT; i++) {
      sum += buffer[i];
    }
    return sum / SMOOTH_COUNT;
  }

public:
  HTS221(uint8_t addr = HTS221_ADDR) : address(addr) {}

  bool begin() {
    Wire.begin();
    
    // Verify device ID
    if (readRegister(WHO_AM_I_REG) != 0xBC) {
      return false;
    }

    // Configure sensor
    Wire.beginTransmission(address);
    Wire.write(CTRL_REG1);
    Wire.write(0x85);  // Power on, BDU enabled, 12.5 Hz
    Wire.endTransmission();

    // Read calibration data
    uint8_t T0_degC_x8 = readRegister(CALIB_T0_DEGC_X8);
    uint8_t T1_degC_x8 = readRegister(CALIB_T1_DEGC_X8);
    uint8_t T0_T1_msb = readRegister(CALIB_T0_T1_MSB);

    calib.T0_degC = ((T0_T1_msb & 0x03) << 8 | T0_degC_x8) / 8.0f;
    calib.T1_degC = ((T0_T1_msb & 0x0C) << 6 | T1_degC_x8) / 8.0f;
    calib.T0_out = readInt16(CALIB_T0_OUT_L, CALIB_T0_OUT_H);
    calib.T1_out = readInt16(CALIB_T1_OUT_L, CALIB_T1_OUT_H);
    calib.H0_rh = readRegister(CALIB_H0_RH_X2) / 2.0f;
    calib.H1_rh = readRegister(CALIB_H1_RH_X2) / 2.0f;
    calib.H0_T0_out = readInt16(CALIB_H0_T0_OUT_L, CALIB_H0_T0_OUT_H);
    calib.H1_T0_out = readInt16(CALIB_H1_T0_OUT_L, CALIB_H1_T0_OUT_H);

    // Initialize smoothing buffers
    for (uint8_t i = 0; i < SMOOTH_COUNT; i++) {
      tempBuffer[i] = calib.T0_degC;
      humBuffer[i] = calib.H0_rh;
    }

    return true;
  }

  void readData(float &temperature, float &humidity) {
    int16_t temp_raw = readInt16(TEMP_OUT_L, TEMP_OUT_H);
    temperature = calib.T0_degC + (float)(temp_raw - calib.T0_out) * 
                 (calib.T1_degC - calib.T0_degC) / (float)(calib.T1_out - calib.T0_out);

    int16_t hum_raw = readInt16(HUMIDITY_OUT_L, HUMIDITY_OUT_H);
    humidity = calib.H0_rh + (float)(hum_raw - calib.H0_T0_out) * 
              (calib.H1_rh - calib.H0_rh) / (float)(calib.H1_T0_out - calib.H0_T0_out);
    humidity = constrain(humidity, 0.0f, 100.0f);

    // Apply smoothing
    temperature = smooth(tempBuffer, temperature);
    humidity = smooth(humBuffer, humidity);
  }

  const CalibrationData& getCalibrationData() const {
    return calib;
  }
};

// Initialize the sensor
HTS221 hts221;

// Function to send data to proxy server
void sendDataToProxy() {
  float temperature, humidity;
  hts221.readData(temperature, humidity);

  // Create the JSON payload
  char payload[128];
  snprintf(payload, sizeof(payload), 
           "{\"temperature\":%.2f,\"humidity\":%.1f,\"timestamp\":%lu}",
           temperature, humidity, millis());

  if (client.connect(proxyHost, proxyPort)) {
    Serial.println("Connected to proxy server");

    // Send HTTP POST request
    char request[256];
    snprintf(request, sizeof(request),
             "POST /sensor-data HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Connection: close\r\n"
             "Content-Length: %d\r\n"
             "\r\n"
             "%s",
             proxyHost, strlen(payload), payload);
    
    Serial.println("Sending payload:");
    Serial.println(payload);
    
    client.print(request);

    // Wait for response with timeout
    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println(">>> Client Timeout!");
        client.stop();
        return;
      }
      delay(10);
    }

    // Read and print the response
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
    
    client.stop();
  } else {
    Serial.println("Failed to connect to proxy server");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial connection on USB-native boards

  // Connect to WiFi
  if (WiFi.begin(ssid, password) != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
  }
  Serial.println("Connected to WiFi!");

  // Print WiFi connection details
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Signal Strength (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  if (!hts221.begin()) {
    Serial.println("Error: HTS221 sensor not found!");
    while(1);
  }

  Serial.println("HTS221 Initialized successfully!");
}

void loop() {
  float temperature, humidity;
  hts221.readData(temperature, humidity);

  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.print(" °C\tHumidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");

  // Send data to proxy server
  sendDataToProxy();

  delay(10000);  // Update every 10 seconds
}
