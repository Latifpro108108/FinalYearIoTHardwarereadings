#include <MXChipFirebase.h>
#include "HTS221Sensor.h"
#include "Wire.h"

// Initialize sensor objects
DevI2C i2c(D14, D15);
HTS221Sensor sensor(&i2c);
MXChipFirebase firebase;

// Firebase configuration
const char* FIREBASE_URL = "mxchipreadings-default-rtdb.firebaseio.com";
const char* FIREBASE_API_KEY = "AIzaSyAGDZJ7sFzUQcbdU6pRlybuEcwtMRc1aFU";

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("Starting Humidity Monitoring");

  // Initialize sensor
  sensor.init(NULL);
  sensor.enable();
  
  // Connect to WiFi
  WiFi.begin();  // Uses saved credentials
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  // Initialize Firebase
  firebase.begin(FIREBASE_URL, FIREBASE_API_KEY);
  firebase.setPath("/humidity_readings");
  firebase.setDebugMode(true);
  
  Serial.println("Setup complete!");
}

void loop() {
  float humidity;
  
  // Read humidity data
  sensor.getHumidity(&humidity);
  
  // Print reading to serial
  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");
  
  // Create JSON payload
  char jsonData[64];
  snprintf(jsonData, sizeof(jsonData),
           "{\"humidity\":%.1f,\"timestamp\":%lu,\"device\":\"mxchip_01\"}",
           humidity, millis());
  
  // Send to Firebase
  if (firebase.sendJSON(jsonData)) {
    Serial.println("Data sent successfully!");
  } else {
    Serial.print("Error: ");
    Serial.println(firebase.getLastError());
  }
  
  delay(5000);  // 5 seconds
} 