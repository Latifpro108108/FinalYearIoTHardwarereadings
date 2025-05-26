# MXChip Firebase Bridge

A library for connecting MXChip AZ3166 devices directly to Firebase Realtime Database. This library provides a simple and efficient way to send sensor data from your MXChip device to Firebase without requiring an intermediate proxy server.

## Features

- Direct connection to Firebase Realtime Database
- Simple API for sending data
- Support for multiple data types (float, int, string)
- Built-in JSON formatting
- Debug mode for troubleshooting
- Error handling and status reporting

## Installation

1. Clone this repository into your project's `lib` directory:
```bash
cd your_project/lib
git clone https://github.com/Latifpro108108/MXChip-Firebase-Bridge.git
```

2. Add the following to your `platformio.ini`:
```ini
lib_deps = 
    EMW10150Lib
    SystemWiFi
    AZ3166WiFi
    Microsoft Azure IoT SDK
    AZ3166
    HTTPClient
```

## Usage

```cpp
#include <MXChipFirebase.h>

// Initialize the library
MXChipFirebase firebase;

void setup() {
    // Connect to WiFi first
    WiFi.begin();
    
    // Initialize Firebase with your credentials
    firebase.begin("your-project.firebaseio.com", "your-api-key");
    firebase.setPath("/sensor_data");
    firebase.setDebugMode(true);
}

void loop() {
    // Send data to Firebase
    firebase.send("temperature", 25.5);
    firebase.send("humidity", 60);
    firebase.send("status", "active");
    
    delay(5000);
}
```

## API Reference

### Constructor
- `MXChipFirebase()` - Creates a new instance of the MXChipFirebase class

### Configuration Methods
- `void begin(const char* databaseURL, const char* apiKey)` - Initialize Firebase connection
- `void setPath(const char* path)` - Set the database path for data storage
- `void setDebugMode(bool enable)` - Enable/disable debug output

### Data Transmission Methods
- `bool send(const char* key, float value)` - Send a float value
- `bool send(const char* key, int value)` - Send an integer value
- `bool send(const char* key, const char* value)` - Send a string value
- `bool sendJSON(const char* jsonString)` - Send a custom JSON string

### Status Methods
- `bool isConnected()` - Check if connected to WiFi
- `const char* getLastError()` - Get the last error message

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details

## Author

Abdoul Latif Dabone

## Acknowledgments

- Microsoft MXChip Team
- Firebase Team
- Arduino Community 