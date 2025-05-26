#include "MXChipFirebase.h"

MXChipFirebase::MXChipFirebase() {
    _debugMode = false;
    _path = "";
}

void MXChipFirebase::begin(const char* databaseURL, const char* apiKey) {
    _databaseURL = String(databaseURL);
    _apiKey = String(apiKey);
    
    // Remove https:// if present
    if (_databaseURL.startsWith("https://")) {
        _databaseURL = _databaseURL.substring(8);
    }
    
    if (_debugMode) {
        Serial.println("Firebase initialized with:");
        Serial.print("Database URL: ");
        Serial.println(_databaseURL);
        Serial.print("API Key: ");
        Serial.println("********");
    }
}

void MXChipFirebase::setPath(const char* path) {
    _path = String(path);
    if (!_path.startsWith("/")) {
        _path = "/" + _path;
    }
}

void MXChipFirebase::setDebugMode(bool enable) {
    _debugMode = enable;
}

String MXChipFirebase::buildRequestURL(const String& path) {
    String url = _databaseURL + path;
    if (!url.endsWith(".json")) {
        url += ".json";
    }
    url += "?auth=" + _apiKey;
    return url;
}

bool MXChipFirebase::makeRequest(const char* method, const String& path, const String& data) {
    if (_debugMode) {
        Serial.print("Making request to: ");
        Serial.println(_databaseURL);
        Serial.print("Path: ");
        Serial.println(path);
        Serial.print("Method: ");
        Serial.println(method);
        Serial.print("Data: ");
        Serial.println(data);
    }

    // Connect to Firebase
    if (!_client.connect(_databaseURL.c_str(), 443)) {
        _lastError = "Connection failed";
        return false;
    }

    // Build the HTTP request
    String request = String(method) + " " + path;
    if (!path.endsWith(".json")) {
        request += ".json";
    }
    request += "?auth=" + _apiKey + " HTTP/1.1\r\n";
    request += "Host: " + _databaseURL + "\r\n";
    request += "Connection: close\r\n";
    request += "Content-Type: application/json\r\n";
    
    if (data.length() > 0) {
        request += "Content-Length: " + String(data.length()) + "\r\n";
    }
    request += "\r\n";
    
    if (data.length() > 0) {
        request += data;
    }

    // Send the request
    _client.print(request);

    // Read the response
    String response = "";
    int statusCode = -1;
    
    while (_client.connected()) {
        String line = _client.readStringUntil('\n');
        if (line.startsWith("HTTP/1.1")) {
            statusCode = line.substring(9, 12).toInt();
        }
        if (line == "\r") {
            break;
        }
    }

    // Read the response body
    while (_client.available()) {
        response += (char)_client.read();
    }

    _client.stop();

    if (_debugMode) {
        Serial.print("Status code: ");
        Serial.println(statusCode);
        Serial.print("Response: ");
        Serial.println(response);
    }

    bool success = (statusCode >= 200 && statusCode < 300);
    if (!success) {
        _lastError = response;
    }
    return success;
}

bool MXChipFirebase::send(const char* key, float value) {
    char valueStr[20];
    dtostrf(value, 4, 2, valueStr);
    return send(key, valueStr);
}

bool MXChipFirebase::send(const char* key, int value) {
    char valueStr[20];
    itoa(value, valueStr, 10);
    return send(key, valueStr);
}

bool MXChipFirebase::send(const char* key, const char* value) {
    String json = buildJSON(key, value);
    return sendJSON(json.c_str());
}

bool MXChipFirebase::sendJSON(const char* jsonString) {
    return makeRequest("POST", _path, jsonString);
}

bool MXChipFirebase::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

const char* MXChipFirebase::getLastError() {
    return _lastError.c_str();
}

String MXChipFirebase::floatToString(float value, int precision) {
    char buffer[20];
    dtostrf(value, 4, precision, buffer);
    return String(buffer);
}

String MXChipFirebase::buildJSON(const char* key, const char* value) {
    String json = "{\"";
    json += key;
    json += "\":\"";
    json += value;
    json += "\",\"timestamp\":";
    json += millis();
    json += "}";
    return json;
} 