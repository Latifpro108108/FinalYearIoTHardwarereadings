#include <MXChipFirebase.h>
#include "HTS221Sensor.h"
#include "Wire.h"

// WiFi credentials
const char* ssid = "Dabone's";     
const char* password = "12345679"; 

// Proxy server details
const char* proxyIP = "100.100.100.171";  // Your computer's IP address
const int proxyPort = 3000;

// Initialize objects
MXChipFirebase firebase;
DevI2C i2c(D14, D15);
HTS221Sensor sensor(&i2c);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Starting Humidity Monitoring");

  // Initialize sensor
  sensor.init(NULL);
  sensor.enable();
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  // Initialize Firebase connection
  firebase.begin(proxyIP, proxyPort);
  firebase.setDebugMode(true);
  firebase.setPath("/humidity_readings"); // Set specific path for humidity data
  
  Serial.println("Setup complete!");
}

void loop() {
  float humidity;
  
  // Read only humidity data
  sensor.getHumidity(&humidity);
  
  // Print reading to serial
  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");
  
  // Create JSON payload with only humidity
  char jsonData[64];
  snprintf(jsonData, sizeof(jsonData),
           "{\"humidity\":%.1f,\"timestamp\":%lu,\"device\":\"mxchip_01\"}",
           humidity, millis());
  
  // Send to Firebase via proxy
  if (firebase.sendJSON(jsonData)) {
    Serial.println("Humidity data sent successfully!");
  } else {
    Serial.println("Failed to send humidity data!");
  }
  
  // Wait before next reading
  delay(5000);  // 5 seconds
} 