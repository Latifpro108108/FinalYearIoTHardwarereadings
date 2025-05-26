#ifndef MXCHIP_FIREBASE_H
#define MXCHIP_FIREBASE_H

#include <Arduino.h>
#include "AZ3166WiFi.h"
#include "SystemWiFi.h"
#include "Wire.h"

class MXChipFirebase {
private:
    String _databaseURL;
    String _apiKey;
    String _path;
    WiFiSSLClient _client;
    
    bool _debugMode;
    String _lastError;
    
    // Internal methods
    bool makeRequest(const char* method, const String& path, const String& data);
    String buildRequestURL(const String& path);
    
public:
    MXChipFirebase();
    
    // Basic configuration - only need Firebase credentials
    void begin(const char* databaseURL, const char* apiKey);
    void setPath(const char* path);
    void setDebugMode(bool enable);
    
    // Data transmission methods
    bool send(const char* key, float value);
    bool send(const char* key, int value);
    bool send(const char* key, const char* value);
    bool sendJSON(const char* jsonString);
    
    // Status methods
    bool isConnected();
    const char* getLastError();
    
    // Utility methods
    static String floatToString(float value, int precision = 2);
    static String buildJSON(const char* key, const char* value);
};

#endif // MXCHIP_FIREBASE_H 