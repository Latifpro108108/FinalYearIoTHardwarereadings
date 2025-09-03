# Code Documentation - Mental Health Monitoring System

## 📋 Table of Contents
1. [Project Structure](#project-structure)
2. [Main Code Analysis](#main-code-analysis)
3. [Sensor Implementation Details](#sensor-implementation-details)
4. [Data Processing Algorithms](#data-processing-algorithms)
5. [Display System](#display-system)
6. [Error Handling](#error-handling)
7. [Performance Optimization](#performance-optimization)

---

## 🏗️ Project Structure

```
FinalYearIoTHardwarereadings/
├── src/
│   └── main.cpp                 # Main application code
├── lib/
│   └── MXChipFirebase/          # Custom Firebase library
├── include/
│   └── README                   # Include directory info
├── test/
│   └── README                   # Test directory info
├── platformio.ini              # PlatformIO configuration
├── README.md                   # Project documentation
├── CODE_DOCUMENTATION.md       # This file
└── TECHNICAL_DOCUMENTATION.md  # Technical implementation details
```

---

## 💻 Main Code Analysis

### File: `src/main.cpp`

This is the core application file containing all sensor control, data processing, and display logic.

#### Key Sections:

1. **Header Includes and Definitions**
2. **I2C Communication Functions**
3. **Sensor Classes (HTS221, LSM6DS3)**
4. **Data Processing Classes**
5. **Display System**
6. **Main Setup and Loop Functions**

---

## 🔧 Sensor Implementation Details

### 1. HTS221 Temperature & Humidity Sensor

#### Class: `HTS221_Direct`

**Purpose**: Direct hardware control of temperature and humidity sensor

**Key Methods**:
- `begin()`: Initialize sensor with calibration data
- `readData()`: Read temperature and humidity values
- `smoothData()`: Apply 5-point moving average smoothing

**Technical Details**:
```cpp
// Calibration data structure
struct HTS221_Calibration {
    float T0_degC, T1_degC;      // Temperature calibration points
    int16_t T0_out, T1_out;      // Raw output calibration points
    float H0_rh, H1_rh;          // Humidity calibration points
    int16_t H0_T0_out, H1_T0_out; // Humidity raw output points
};
```

**Initialization Sequence**:
1. Verify device identity (WHO_AM_I = 0xBC)
2. Configure control registers (12.5Hz, BDU=1, PD=1)
3. Read calibration data from sensor memory
4. Initialize smoothing buffers

**Data Reading Process**:
1. Check status register for new data
2. Read raw 16-bit temperature and humidity values
3. Apply calibration formula for accurate readings
4. Apply smoothing filter for stability

### 2. LSM6DS3 Motion Sensor

#### Class: `LSM6DS3_Direct`

**Purpose**: Direct hardware control of 6-axis motion sensor (accelerometer + gyroscope)

**Key Methods**:
- `begin()`: Initialize sensor with robust configuration
- `readData()`: Read accelerometer and gyroscope data
- Motion magnitude calculation and detection

**Technical Details**:
```cpp
// Motion data structure
struct MotionData {
    float accelX, accelY, accelZ;    // Accelerometer data (m/s²)
    float gyroX, gyroY, gyroZ;       // Gyroscope data (degrees/s)
    float motionMagnitude;            // Overall motion level
    bool isMoving;                    // Motion detection flag
    bool sensorWorking;               // Sensor status flag
};
```

**Initialization Sequence**:
1. Check device identity (WHO_AM_I = 0x69 or 0x6A)
2. Perform complete device reset
3. Configure accelerometer (100Hz, ±2g)
4. Configure gyroscope (100Hz, ±245dps)
5. Enable data output with BDU (Block Data Update)
6. Test data production with multiple attempts
7. Fallback to alternative configuration if needed

**Data Processing**:
1. Read 6 bytes of accelerometer data
2. Read 6 bytes of gyroscope data
3. Convert raw values to physical units
4. Calculate motion magnitude using vector mathematics
5. Determine motion status based on thresholds

**Scale Factors**:
- Accelerometer: 0.061 mg/LSB × 0.001 × 9.81 = m/s²
- Gyroscope: 8.75 mdps/LSB × 0.001 = degrees/s

### 3. Microphone Implementation

**Purpose**: Sound level monitoring using analog input

**Implementation**:
```cpp
#define MIC_PIN A3  // Analog pin A3 for microphone input

// Read microphone with averaging
int micSum = 0;
for (int i = 0; i < 3; i++) {
    micSum += analogRead(MIC_PIN);
    delay(5);
}
int micValue = micSum / 3;
```

**Features**:
- 3-sample averaging for noise reduction
- 0-1023 range (10-bit ADC)
- 5ms delays between samples for stability

---

## 📊 Data Processing Algorithms

### 1. Intelligent Sensor Monitor

#### Class: `IntelligentSensorMonitor`

**Purpose**: Advanced data analysis and pattern recognition

**Key Features**:
- **Data Buffering**: 10-sample circular buffer
- **Trend Analysis**: Rate of change calculation
- **Alert Generation**: Threshold-based warnings
- **Health Assessment**: Overall status evaluation

**Data Structures**:
```cpp
struct SensorData {
    float temperature;
    float humidity;
    float motionMagnitude;
    float soundLevel;
    unsigned long timestamp;
};

struct AnalysisResult {
    String soundStatus;
    bool soundAlert;
    int soundViolationCount;
    String motionStatus;
    bool motionAlert;
    int motionViolationCount;
    String tempStatus;
    String humidityStatus;
    bool environmentalAlert;
    String overallStatus;
    int alertLevel; // 0=Normal, 1=Warning, 2=Alert, 3=Critical
};
```

**Analysis Algorithms**:

#### Sound Analysis:
```cpp
String analyzeSound(float soundLevel) {
    if (soundLevel <= 50) return "SILENCE";
    if (soundLevel <= 100) return "LOW";
    if (soundLevel <= 200) return "MEDIUM";
    if (soundLevel <= 400) return "HIGH";
    return "DANGEROUS";
}
```

#### Motion Analysis:
```cpp
String analyzeMotion(float motionMagnitude) {
    if (motionMagnitude <= 2.0) return "CALM";
    if (motionMagnitude <= 5.0) return "NORMAL";
    if (motionMagnitude <= 10.0) return "ACTIVE";
    return "VIOLENT";
}
```

#### Trend Calculation:
```cpp
void calculateTrends() {
    if (bufferIndex >= 2) {
        int prevIndex = (bufferIndex - 2 + SMOOTHING_SAMPLES) % SMOOTHING_SAMPLES;
        int currIndex = (bufferIndex - 1 + SMOOTHING_SAMPLES) % SMOOTHING_SAMPLES;
        
        unsigned long timeDiff = dataBuffer[currIndex].timestamp - dataBuffer[prevIndex].timestamp;
        if (timeDiff > 0) {
            tempTrend = (dataBuffer[currIndex].temperature - dataBuffer[prevIndex].temperature) / (timeDiff / 1000.0f);
            // Similar calculations for other sensors
        }
    }
}
```

### 2. Clean Display System

#### Class: `CleanDisplay`

**Purpose**: Human-readable data presentation with professional formatting

**Key Features**:
- **30-second averaging** for stable readings
- **Change detection** with configurable thresholds
- **Professional formatting** with borders and emojis
- **Status summaries** with color-coded indicators

**Display Parameters**:
```cpp
#define DISPLAY_INTERVAL_MS 5000        // Show data every 5 seconds
#define AVERAGE_WINDOW_MS 30000         // Average over 30 seconds
#define SIGNIFICANT_CHANGE_TEMP 1.0     // 1°C change
#define SIGNIFICANT_CHANGE_HUM 3.0      // 3% change
#define SIGNIFICANT_CHANGE_MOTION 1.0   // 1 m/s² change
#define SIGNIFICANT_CHANGE_SOUND 30     // 30 unit change
```

**Display Format**:
```
╔══════════════════════════════════════════════════════════════╗
║                    MENTAL HEALTH MONITOR                    ║
║                    Real-Time Sensor Data                    ║
╚══════════════════════════════════════════════════════════════╝

🕐 Time: 45 seconds | Sample Count: 45

📊 SENSOR READINGS (30-second averages):
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🌡️  Temperature: 24.8°C
💧 Humidity:    62.3%
📱 Motion:      1.85 m/s²
🎤 Sound:       78 units

📋 STATUS SUMMARY:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
🌡️  Temperature: ✅ COMFORTABLE
💧 Humidity:    ✅ COMFORTABLE
📱 Motion:      ✅ NORMAL
🎤 Sound:       ✅ LOW

🏥 OVERALL HEALTH ASSESSMENT:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
✅ ALL SYSTEMS NORMAL - Patient is comfortable
```

---

## ⚠️ Error Handling

### 1. Sensor Initialization Errors

**HTS221 Error Handling**:
```cpp
if (i2cReadRegister(address, HTS221_WHO_AM_I) != 0xBC) {
    Serial.println("HTS221: Device not found!");
    return false;
}
```

**LSM6DS3 Error Handling**:
```cpp
// Multiple attempts with different configurations
for (int attempt = 0; attempt < 5; attempt++) {
    // Test data production
    if (hasData) {
        Serial.println("LSM6DS3: ✅ Data is being produced!");
        break;
    }
    delay(500);
}

// Fallback configuration
if (!hasData) {
    i2cWriteRegister(address, LSM6DS3_CTRL1_XL, 0x60); // 833Hz
    i2cWriteRegister(address, LSM6DS3_CTRL2_G, 0x60); // 833Hz
}
```

### 2. Runtime Error Handling

**Fallback Mechanism**:
```cpp
static bool lsm6ds3_working = true;
if (lsm6ds3_working) {
    lsm6ds3.readData(motion);
    if (!motion.sensorWorking) {
        lsm6ds3_working = false;
        Serial.println("LSM6DS3: Sensor failed during operation - using fallback");
    }
} else {
    // Fallback: simulate minimal motion data
    motion.accelX = 0.0f;
    motion.accelY = 0.0f;
    motion.accelZ = 9.81f; // Gravity only
    motion.motionMagnitude = 9.81f;
    motion.isMoving = false;
    motion.sensorWorking = false;
}
```

### 3. I2C Communication Errors

**Robust I2C Functions**:
```cpp
uint8_t i2cReadRegister(uint8_t deviceAddr, uint8_t reg) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddr, (uint8_t)1);
    return Wire.read();
}
```

---

## 🚀 Performance Optimization

### 1. Memory Management

**Efficient Data Structures**:
- Use of `static` variables for persistent data
- Circular buffers for efficient memory usage
- Minimal string operations to reduce memory fragmentation

**Memory Usage**:
- RAM: 17.1% (44,848 bytes from 262,144 bytes)
- Flash: 21.1% (221,548 bytes from 1,048,576 bytes)

### 2. Timing Optimization

**Sampling Strategy**:
- Raw sampling: 1 second intervals
- Display updates: 5 second intervals
- Analysis windows: 10 second intervals
- Averaging windows: 30 seconds

**Delay Management**:
```cpp
// Minimal delays for sensor stability
delay(100);  // For I2C operations
delay(500);  // For sensor stabilization
delay(1000); // For display updates
```

### 3. Processing Efficiency

**Optimized Calculations**:
- Pre-calculated scale factors
- Efficient vector magnitude calculation
- Minimal floating-point operations
- Smart averaging algorithms

---

## 🔍 Debugging Features

### 1. I2C Bus Scanner

```cpp
// Scan I2C bus to identify connected devices
for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
        Serial.print("I2C device found at address 0x");
        Serial.println(addr, 16);
    }
}
```

### 2. Detailed Sensor Debugging

**LSM6DS3 Debug Output**:
```
LSM6DS3: Starting ROBUST initialization...
LSM6DS3: Checking device ID...
LSM6DS3: Attempt 1 - Device ID = 0x6A
LSM6DS3: ✅ Device found with ID 0x6A
LSM6DS3: Performing complete reset...
LSM6DS3: Reset status = 0x04
LSM6DS3: Configuring accelerometer...
LSM6DS3: Configuring gyroscope...
LSM6DS3: Configuring control register...
LSM6DS3: CTRL1_XL = 0x50
LSM6DS3: CTRL2_G = 0x50
LSM6DS3: CTRL3_C = 0x04
LSM6DS3: Waiting for stabilization...
LSM6DS3: Testing data production...
LSM6DS3: Attempt 1 - Test read: 0x12 0x34 0x56 0x78 0x9A 0xBC
LSM6DS3: ✅ Data is being produced!
LSM6DS3: ✅ Initialization successful!
```

### 3. Configuration Verification

**Register Read-back**:
```cpp
uint8_t ctrl1 = i2cReadRegister(address, LSM6DS3_CTRL1_XL);
uint8_t ctrl2 = i2cReadRegister(address, LSM6DS3_CTRL2_G);
uint8_t ctrl3 = i2cReadRegister(address, LSM6DS3_CTRL3_C);

Serial.print("LSM6DS3: CTRL1_XL = 0x");
Serial.println(ctrl1, 16);
```

---

## 📈 Code Quality Metrics

### 1. Code Organization
- **Modular design** with separate classes for each sensor
- **Clear separation** of concerns (sensors, processing, display)
- **Consistent naming** conventions
- **Comprehensive comments** and documentation

### 2. Error Handling
- **Graceful degradation** when sensors fail
- **Multiple fallback** mechanisms
- **Detailed error messages** for debugging
- **Robust initialization** sequences

### 3. Performance
- **Efficient memory usage** with circular buffers
- **Optimized timing** for real-time operation
- **Minimal processing overhead**
- **Stable data output** with averaging

---

## 🎯 Key Technical Achievements

1. **Direct Hardware Control**: Bypassed library limitations for maximum reliability
2. **Robust Error Handling**: Graceful degradation when sensors fail
3. **Professional Data Display**: Clean, readable output suitable for medical use
4. **Intelligent Processing**: Smart averaging and change detection algorithms
5. **Comprehensive Debugging**: Detailed output for troubleshooting

---

*This code represents a professional-grade implementation of IoT-based mental health monitoring, demonstrating advanced embedded programming techniques and real-time data processing capabilities.*
