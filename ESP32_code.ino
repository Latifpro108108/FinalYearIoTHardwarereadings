#include <WiFi.h>

// WiFi Configuration
const char* ssid = "MXCHIP_AP";         // MXCHIP's AP name
const char* password = "12345678";       // MXCHIP's AP password

// IP Configuration
IPAddress staticIP(192, 168, 4, 2);     // Static IP for ESP32
IPAddress gateway(192, 168, 4, 1);      // Gateway (MXCHIP's IP)
IPAddress subnet(255, 255, 255, 0);     // Subnet mask

// Web Server
WiFiServer server(80);

// LED Configuration
const int LED_PIN = 2;                   // GPIO2 on ESP32
bool ledState = LOW;

// Connection Management
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000;  // 5 seconds between reconnection attempts
const unsigned long CONNECTION_TIMEOUT = 10000;  // 10 seconds connection timeout

void connectToWiFi() {
  Serial.printf("\nConnecting to %s...\n", ssid);
  
  WiFi.mode(WIFI_STA);      // Set WiFi to station mode
  WiFi.disconnect(true);    // Disconnect from any previous WiFi
  delay(1000);             // Wait for WiFi mode change
  
  // Configure static IP before connecting
  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("Static IP Configuration Failed!");
  }
  
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=== WiFi Connected! ===");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Subnet: ");
    Serial.println(WiFi.subnetMask());
    
    // Start web server
    server.begin();
    Serial.println("Server started - Ready for commands!");
  } else {
    Serial.println("\nConnection Failed!");
  }
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32 Starting ===");
  
  // Configure LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);
  
  // Initial connection
  connectToWiFi();
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost!");
    delay(5000);  // Wait 5 seconds before trying to reconnect
    connectToWiFi();
    return;
  }
  
  // Handle incoming clients
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("New client connected");
    String currentLine = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Send response headers
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/plain");
            client.println("Access-Control-Allow-Origin: *");
            client.println("Connection: close");
            client.println();
            
            // Send LED state
            client.printf("LED is %s\n", ledState ? "ON" : "OFF");
            break;
          } else {
            if (currentLine.indexOf("GET /toggle") >= 0) {
              ledState = !ledState;
              digitalWrite(LED_PIN, ledState);
              Serial.printf("LED toggled: %s\n", ledState ? "ON" : "OFF");
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
} 