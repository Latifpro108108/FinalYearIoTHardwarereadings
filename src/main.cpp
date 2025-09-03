#include <Arduino.h>
#include "Wire.h"

// ============================================================================
// DIRECT HARDWARE SENSOR IMPLEMENTATION
// ============================================================================

// I2C Addresses for Built-in Sensors
#define HTS221_ADDR        0x5F    // Temperature & Humidity
#define LSM6DS3_ADDR       0x6A    // Accelerometer & Gyroscope  
#define LPS22HB_ADDR       0x5C    // Barometric Pressure
#define LIS2MDL_ADDR       0x1E    // Magnetometer

// HTS221 Register Map (Temperature & Humidity)
#define HTS221_WHO_AM_I        0x0F
#define HTS221_CTRL_REG1       0x20
#define HTS221_CTRL_REG2       0x21
#define HTS221_CTRL_REG3       0x22
#define HTS221_STATUS_REG      0x27
#define HTS221_TEMP_OUT_L      0x2A
#define HTS221_TEMP_OUT_H      0x2B
#define HTS221_HUMIDITY_OUT_L  0x28
#define HTS221_HUMIDITY_OUT_H  0x29
#define HTS221_CALIB_T0_DEGC_X8    0x32
#define HTS221_CALIB_T1_DEGC_X8    0x33
#define HTS221_CALIB_T0_T1_MSB     0x35
#define HTS221_CALIB_T0_OUT_L      0x3C
#define HTS221_CALIB_T0_OUT_H      0x3D
#define HTS221_CALIB_T1_OUT_L      0x3E
#define HTS221_CALIB_T1_OUT_H      0x3F
#define HTS221_CALIB_H0_RH_X2      0x30
#define HTS221_CALIB_H1_RH_X2      0x31
#define HTS221_CALIB_H0_T0_OUT_L   0x36
#define HTS221_CALIB_H0_T0_OUT_H   0x37
#define HTS221_CALIB_H1_T0_OUT_L   0x3A
#define HTS221_CALIB_H1_T0_OUT_H   0x3B

// LSM6DS3 Register Map (Accelerometer & Gyroscope)
#define LSM6DS3_WHO_AM_I       0x0F
#define LSM6DS3_CTRL1_XL       0x10
#define LSM6DS3_CTRL2_G        0x11
#define LSM6DS3_CTRL3_C        0x12
#define LSM6DS3_CTRL4_C        0x13
#define LSM6DS3_CTRL5_C        0x14
#define LSM6DS3_CTRL6_C        0x15
#define LSM6DS3_CTRL7_G        0x16
#define LSM6DS3_CTRL8_XL       0x17
#define LSM6DS3_CTRL9_XL       0x18
#define LSM6DS3_CTRL10_C       0x19
#define LSM6DS3_OUTX_L_XL      0x28
#define LSM6DS3_OUTX_H_XL      0x29
#define LSM6DS3_OUTY_L_XL      0x2A
#define LSM6DS3_OUTY_H_XL      0x2B
#define LSM6DS3_OUTZ_L_XL      0x2C
#define LSM6DS3_OUTZ_H_XL      0x2D
#define LSM6DS3_OUTX_L_G       0x22
#define LSM6DS3_OUTX_H_G       0x23
#define LSM6DS3_OUTY_L_G       0x24
#define LSM6DS3_OUTY_H_G       0x25
#define LSM6DS3_OUTZ_L_G       0x26
#define LSM6DS3_OUTZ_H_G       0x27

// LPS22HB Register Map (Pressure)
#define LPS22HB_WHO_AM_I       0x0F
#define LPS22HB_CTRL_REG1      0x10
#define LPS22HB_CTRL_REG2      0x11
#define LPS22HB_CTRL_REG3      0x12
#define LPS22HB_STATUS_REG     0x27
#define LPS22HB_PRESS_OUT_XL   0x28
#define LPS22HB_PRESS_OUT_L    0x29
#define LPS22HB_PRESS_OUT_H    0x2A
#define LPS22HB_TEMP_OUT_L     0x2B
#define LPS22HB_TEMP_OUT_H     0x2C

// LIS2MDL Register Map (Magnetometer)
#define LIS2MDL_WHO_AM_I       0x4F
#define LIS2MDL_CFG_REG_A      0x60
#define LIS2MD3_CFG_REG_C      0x62
#define LIS2MDL_STATUS_REG     0x67
#define LIS2MDL_OUTX_L_REG     0x68
#define LIS2MDL_OUTX_H_REG     0x69
#define LIS2MDL_OUTY_L_REG     0x6A
#define LIS2MDL_OUTY_H_REG     0x6B
#define LIS2MDL_OUTZ_L_REG     0x6C
#define LIS2MDL_OUTZ_H_REG     0x6D

// Microphone Pin (Analog)
#define MIC_PIN A3

// Sound Calibration Parameters
#define SOUND_BASELINE_SAMPLES 50    // Samples to take for baseline calibration
#define SOUND_BASELINE_THRESHOLD 5   // Minimum change from baseline to register as sound

// ============================================================================
// DIRECT I2C COMMUNICATION FUNCTIONS
// ============================================================================

// Direct I2C Write - Single Register
void i2cWriteRegister(uint8_t deviceAddr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Direct I2C Read - Single Register
uint8_t i2cReadRegister(uint8_t deviceAddr, uint8_t reg) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddr, (uint8_t)1);
    return Wire.read();
  }

// Direct I2C Read - Multiple Registers
void i2cReadRegisters(uint8_t deviceAddr, uint8_t reg, uint8_t* data, uint8_t length) {
    Wire.beginTransmission(deviceAddr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(deviceAddr, length);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }
}

// Direct I2C Read - 16-bit Value
int16_t i2cRead16Bit(uint8_t deviceAddr, uint8_t regL, uint8_t regH) {
    uint8_t low = i2cReadRegister(deviceAddr, regL);
    uint8_t high = i2cReadRegister(deviceAddr, regH);
    return (int16_t)((high << 8) | low);
  }

// ============================================================================
// SOUND SENSOR CALIBRATION SYSTEM
// ============================================================================

class SoundCalibrator {
private:
    int baselineValue;
    bool isCalibrated;
    int calibrationSamples;
    
public:
    SoundCalibrator() {
        baselineValue = 0;
        isCalibrated = false;
        calibrationSamples = 0;
    }
    
    // Calibrate baseline during quiet period
    void calibrate() {
        Serial.println("🎤 Sound Sensor: Starting baseline calibration...");
        Serial.println("🎤 Please keep quiet for 5 seconds...");
        
        long sum = 0;
        int samples = 0;
        
        for (int i = 0; i < SOUND_BASELINE_SAMPLES; i++) {
            sum += analogRead(MIC_PIN);
            samples++;
            delay(100);
            
            if (i % 10 == 0) {
                Serial.print("🎤 Calibrating... ");
                Serial.print((i * 100) / SOUND_BASELINE_SAMPLES);
                Serial.println("%");
            }
        }
        
        baselineValue = sum / samples;
        isCalibrated = true;
        
        Serial.print("🎤 Baseline calibrated: ");
        Serial.print(baselineValue);
        Serial.println(" units");
        Serial.println("🎤 Sound sensor ready!");
    }
    
    // Get calibrated sound level (0 = silence, positive = sound detected)
    int getCalibratedSoundLevel() {
        if (!isCalibrated) {
            return analogRead(MIC_PIN); // Return raw value if not calibrated
        }
        
        int rawValue = analogRead(MIC_PIN);
        int calibratedValue = rawValue - baselineValue;
        
        // Only return positive values (sound above baseline)
        return (calibratedValue > 0) ? calibratedValue : 0;
    }
    
    bool isReady() {
        return isCalibrated;
    }
    
    int getBaseline() {
        return baselineValue;
    }
};

// Global sound calibrator
SoundCalibrator soundCalibrator;

// ============================================================================
// HTS221 TEMPERATURE & HUMIDITY SENSOR
// ============================================================================

struct HTS221_Calibration {
    float T0_degC, T1_degC;
    int16_t T0_out, T1_out;
    float H0_rh, H1_rh;
    int16_t H0_T0_out, H1_T0_out;
};

class HTS221_Direct {
private:
    uint8_t address;
    HTS221_Calibration calib;
    float tempBuffer[5] = {0};
    float humBuffer[5] = {0};
    uint8_t bufferIndex = 0;

    float smoothData(float* buffer, float newValue) {
        buffer[bufferIndex] = newValue;
        bufferIndex = (bufferIndex + 1) % 5;
    
    float sum = 0;
        for (uint8_t i = 0; i < 5; i++) {
      sum += buffer[i];
    }
        return sum / 5.0f;
  }

public:
    HTS221_Direct(uint8_t addr = HTS221_ADDR) : address(addr) {}

  bool begin() {
    Wire.begin();
    
        // Check device ID
        if (i2cReadRegister(address, HTS221_WHO_AM_I) != 0xBC) {
            Serial.println("HTS221: Device not found!");
      return false;
    }

        // Power on and set data rate
        i2cWriteRegister(address, HTS221_CTRL_REG1, 0x85); // 12.5Hz, BDU=1, ODR=01
        
        // Wait for sensor to stabilize
        delay(100);

    // Read calibration data
        uint8_t T0_degC_x8 = i2cReadRegister(address, HTS221_CALIB_T0_DEGC_X8);
        uint8_t T1_degC_x8 = i2cReadRegister(address, HTS221_CALIB_T1_DEGC_X8);
        uint8_t T0_T1_msb = i2cReadRegister(address, HTS221_CALIB_T0_T1_MSB);

    calib.T0_degC = ((T0_T1_msb & 0x03) << 8 | T0_degC_x8) / 8.0f;
    calib.T1_degC = ((T0_T1_msb & 0x0C) << 6 | T1_degC_x8) / 8.0f;
        calib.T0_out = i2cRead16Bit(address, HTS221_CALIB_T0_OUT_L, HTS221_CALIB_T0_OUT_H);
        calib.T1_out = i2cRead16Bit(address, HTS221_CALIB_T1_OUT_L, HTS221_CALIB_T1_OUT_H);
        calib.H0_rh = i2cReadRegister(address, HTS221_CALIB_H0_RH_X2) / 2.0f;
        calib.H1_rh = i2cReadRegister(address, HTS221_CALIB_H1_RH_X2) / 2.0f;
        calib.H0_T0_out = i2cRead16Bit(address, HTS221_CALIB_H0_T0_OUT_L, HTS221_CALIB_H0_T0_OUT_H);
        calib.H1_T0_out = i2cRead16Bit(address, HTS221_CALIB_H1_T0_OUT_L, HTS221_CALIB_H1_T0_OUT_H);

        // Initialize buffers
        for (uint8_t i = 0; i < 5; i++) {
      tempBuffer[i] = calib.T0_degC;
      humBuffer[i] = calib.H0_rh;
    }

        Serial.println("HTS221: Direct hardware initialization successful!");
    return true;
  }

  void readData(float &temperature, float &humidity) {
        // Check if data is ready
        uint8_t status = i2cReadRegister(address, HTS221_STATUS_REG);
        if (!(status & 0x03)) return; // No new data

        // Read temperature
        int16_t temp_raw = i2cRead16Bit(address, HTS221_TEMP_OUT_L, HTS221_TEMP_OUT_H);
    temperature = calib.T0_degC + (float)(temp_raw - calib.T0_out) * 
                 (calib.T1_degC - calib.T0_degC) / (float)(calib.T1_out - calib.T0_out);

        // Read humidity
        int16_t hum_raw = i2cRead16Bit(address, HTS221_HUMIDITY_OUT_L, HTS221_HUMIDITY_OUT_H);
    humidity = calib.H0_rh + (float)(hum_raw - calib.H0_T0_out) * 
              (calib.H1_rh - calib.H0_rh) / (float)(calib.H1_T0_out - calib.H0_T0_out);
    humidity = constrain(humidity, 0.0f, 100.0f);

    // Apply smoothing
        temperature = smoothData(tempBuffer, temperature);
        humidity = smoothData(humBuffer, humidity);
    }
};

// ============================================================================
// LSM6DS3 ACCELEROMETER & GYROSCOPE SENSOR
// ============================================================================

struct MotionData {
    float accelX, accelY, accelZ;    // m/s²
    float gyroX, gyroY, gyroZ;       // degrees/s
    float motionMagnitude;            // Overall motion level
    bool isMoving;                    // Motion detection flag
    bool sensorWorking;               // Sensor status flag
};

class LSM6DS3_Direct {
private:
    uint8_t address;
    
public:
    LSM6DS3_Direct(uint8_t addr = 0x6A) : address(addr) {}
    
    bool begin() {
        Serial.println("LSM6DS3: Starting ROBUST initialization...");
        
        // 1. Check device ID with multiple attempts
        Serial.println("LSM6DS3: Checking device ID...");
        uint8_t deviceId = 0;
        bool deviceFound = false;
        
        for (int attempt = 0; attempt < 3; attempt++) {
            deviceId = i2cReadRegister(address, LSM6DS3_WHO_AM_I);
            Serial.print("LSM6DS3: Attempt ");
            Serial.print(attempt + 1);
            Serial.print(" - Device ID = 0x");
            Serial.println(deviceId, 16);
            
            if (deviceId == 0x69 || deviceId == 0x6A) {
                deviceFound = true;
                Serial.println("LSM6DS3: ✅ Device found with ID 0x" + String(deviceId, 16));
                break;
            }
            delay(100);
        }
        
        if (!deviceFound) {
            Serial.println("LSM6DS3: Device not found! Expected 0x69 or 0x6A, got 0x" + String(deviceId, 16));
            Serial.println("LSM6DS3: This could be:");
            Serial.println("  - Wrong I2C address (trying 0x6B as alternative)");
            Serial.println("  - Hardware connection issue");
            Serial.println("  - Sensor not powered");
            
            // Try alternative address 0x6B
            Serial.println("LSM6DS3: Trying alternative address 0x6B...");
            address = 0x6B;
            deviceId = i2cReadRegister(address, LSM6DS3_WHO_AM_I);
            Serial.print("LSM6DS3: Alternative address Device ID = 0x");
            Serial.println(deviceId, 16);
            
            if (deviceId != 0x69 && deviceId != 0x6A) {
                Serial.println("LSM6DS3: Alternative address also failed!");
                return false;
            } else {
                Serial.println("LSM6DS3: Found device at alternative address 0x6B!");
            }
        }
        
        // 2. Reset device completely
        Serial.println("LSM6DS3: Performing complete reset...");
        i2cWriteRegister(address, LSM6DS3_CTRL3_C, 0x01);
        delay(200);
        
        // 3. Wait for reset to complete
        uint8_t resetStatus = i2cReadRegister(address, LSM6DS3_CTRL3_C);
        Serial.print("LSM6DS3: Reset status = 0x");
        Serial.println(resetStatus, 16);
        
        // 4. Configure accelerometer: 100Hz, ±2g, BDU enabled
        Serial.println("LSM6DS3: Configuring accelerometer...");
        i2cWriteRegister(address, LSM6DS3_CTRL1_XL, 0x50); // 100Hz, ±2g
        delay(100);
        
        // 5. Configure gyroscope: 100Hz, ±245dps, BDU enabled
        Serial.println("LSM6DS3: Configuring gyroscope...");
        i2cWriteRegister(address, LSM6DS3_CTRL2_G, 0x50); // 100Hz, ±245dps
        delay(100);
        
        // 6. Configure control register: BDU=1, IF_INC=1
        Serial.println("LSM6DS3: Configuring control register...");
        i2cWriteRegister(address, LSM6DS3_CTRL3_C, 0x04); // BDU=1, IF_INC=1
        delay(100);
        
        // 7. Verify configuration
        uint8_t ctrl1 = i2cReadRegister(address, LSM6DS3_CTRL1_XL);
        uint8_t ctrl2 = i2cReadRegister(address, LSM6DS3_CTRL2_G);
        uint8_t ctrl3 = i2cReadRegister(address, LSM6DS3_CTRL3_C);
        
        Serial.print("LSM6DS3: CTRL1_XL = 0x");
        Serial.println(ctrl1, 16);
        Serial.print("LSM6DS3: CTRL2_G = 0x");
        Serial.println(ctrl2, 16);
        Serial.print("LSM6DS3: CTRL3_C = 0x");
        Serial.println(ctrl3, 16);
        
        // 8. Wait for sensor to stabilize
        Serial.println("LSM6DS3: Waiting for stabilization...");
        delay(1000);
        
        // 9. Test data production with multiple attempts
        Serial.println("LSM6DS3: Testing data production...");
        bool hasData = false;
        
        for (int attempt = 0; attempt < 5; attempt++) {
            uint8_t testData[6];
            i2cReadRegisters(address, LSM6DS3_OUTX_L_XL, testData, 6);
            
            Serial.print("LSM6DS3: Attempt ");
            Serial.print(attempt + 1);
            Serial.print(" - Test read: ");
            for (int i = 0; i < 6; i++) {
                Serial.print("0x");
                Serial.print(testData[i], 16);
                Serial.print(" ");
            }
            Serial.println();
            
            // Check if any data is non-zero
            for (int i = 0; i < 6; i++) {
                if (testData[i] != 0x00) {
                    hasData = true;
                    break;
                }
            }
            
            if (hasData) {
                Serial.println("LSM6DS3: ✅ Data is being produced!");
                break;
            }
            
            delay(500);
        }
        
        if (!hasData) {
            Serial.println("LSM6DS3: ❌ No data after multiple attempts - trying alternative config...");
            
            // Try alternative configuration
            i2cWriteRegister(address, LSM6DS3_CTRL1_XL, 0x60); // 833Hz, ±2g
            delay(100);
            i2cWriteRegister(address, LSM6DS3_CTRL2_G, 0x60); // 833Hz, ±245dps
            delay(100);
            i2cWriteRegister(address, LSM6DS3_CTRL3_C, 0x04); // BDU=1
            delay(100);
            
            delay(1000);
            
            // Test again
            uint8_t testData[6];
            i2cReadRegisters(address, LSM6DS3_OUTX_L_XL, testData, 6);
            Serial.print("LSM6DS3: Alternative test: ");
            for (int i = 0; i < 6; i++) {
                Serial.print("0x");
                Serial.print(testData[i], 16);
                Serial.print(" ");
            }
            Serial.println();
            
            hasData = false;
            for (int i = 0; i < 6; i++) {
                if (testData[i] != 0x00) {
                    hasData = true;
                    break;
                }
            }
            
            if (!hasData) {
                Serial.println("LSM6DS3: ❌ Still no data - hardware issue!");
                return false;
            }
        }
        
        Serial.println("LSM6DS3: ✅ Initialization successful!");
        return true;
    }
    
    void readData(MotionData &motion) {
        uint8_t data[12];
        
        // Read accelerometer data
        i2cReadRegisters(address, LSM6DS3_OUTX_L_XL, data, 6);
        
        // Convert to signed 16-bit values
        int16_t accelX_raw = (int16_t)(data[1] << 8 | data[0]);
        int16_t accelY_raw = (int16_t)(data[3] << 8 | data[2]);
        int16_t accelZ_raw = (int16_t)(data[5] << 8 | data[4]);
        
        // Convert to m/s² (scale factor for ±2g range: 0.061 mg/LSB)
        motion.accelX = accelX_raw * 0.061f * 0.001f * 9.81f;
        motion.accelY = accelY_raw * 0.061f * 0.001f * 9.81f;
        motion.accelZ = accelZ_raw * 0.061f * 0.001f * 9.81f;
        
        // Read gyroscope data
        i2cReadRegisters(address, LSM6DS3_OUTX_L_G, data, 6);
        
        // Convert to signed 16-bit values
        int16_t gyroX_raw = (int16_t)(data[1] << 8 | data[0]);
        int16_t gyroY_raw = (int16_t)(data[3] << 8 | data[2]);
        int16_t gyroZ_raw = (int16_t)(data[5] << 8 | data[4]);
        
        // Convert to degrees/s (scale factor for ±245dps range: 8.75 mdps/LSB)
        motion.gyroX = gyroX_raw * 8.75f * 0.001f;
        motion.gyroY = gyroY_raw * 8.75f * 0.001f;
        motion.gyroZ = gyroZ_raw * 8.75f * 0.001f;
        
        // Calculate motion magnitude (excluding gravity)
        // Remove gravity component (assuming Z-axis is vertical)
        float accelX_noGravity = motion.accelX;
        float accelY_noGravity = motion.accelY;
        float accelZ_noGravity = motion.accelZ - 9.81f; // Remove gravity
        
        motion.motionMagnitude = sqrt(accelX_noGravity * accelX_noGravity + 
                                    accelY_noGravity * accelY_noGravity + 
                                    accelZ_noGravity * accelZ_noGravity);
        
        // Motion detection (now properly calibrated)
        motion.isMoving = (motion.motionMagnitude > 0.1f);
        
        // Set sensor working flag
        motion.sensorWorking = true;
    }
};

// ============================================================================
// PROFESSIONAL SENSOR MONITORING SYSTEM
// ============================================================================

// Data Collection Parameters
#define SAMPLE_FREQUENCY_MS 1000        // Sample every 1 second
#define ANALYSIS_WINDOW_MS 10000        // Analyze over 10 seconds
#define SMOOTHING_SAMPLES 10            // Average over 10 samples
#define ALERT_THRESHOLD_COUNT 3         // Alert after 3 consecutive violations

// Sound Level Thresholds (Calibrated Scale - above baseline)
#define SOUND_SILENCE_MAX 5             // 0-5: Silence (near baseline)
#define SOUND_LOW_MAX 20                // 6-20: Low sound
#define SOUND_MEDIUM_MAX 50             // 21-50: Medium sound  
#define SOUND_HIGH_MAX 100              // 51-100: High sound
#define SOUND_DANGEROUS_MIN 100         // 100+: Dangerous/very loud

// Motion Intensity Thresholds (Gravity-Corrected Scale)
#define MOTION_CALM_MAX 0.5             // 0-0.5 m/s²: Calm (gravity-corrected)
#define MOTION_NORMAL_MAX 1.5           // 0.5-1.5 m/s²: Normal movement
#define MOTION_ACTIVE_MAX 3.0           // 1.5-3.0 m/s²: Active movement
#define MOTION_VIOLENT_MIN 3.0          // 3.0+ m/s²: Violent/shaking

// Environmental Thresholds
#define TEMP_COMFORTABLE_MIN 18.0       // 18-26°C: Comfortable
#define TEMP_COMFORTABLE_MAX 26.0
#define TEMP_UNCOMFORTABLE_MIN 26.0     // 26-30°C: Uncomfortable
#define TEMP_UNCOMFORTABLE_MAX 30.0
#define TEMP_DANGEROUS_MIN 30.0         // 30+°C: Dangerous

#define HUMIDITY_COMFORTABLE_MIN 30.0   // 30-70%: Comfortable
#define HUMIDITY_COMFORTABLE_MAX 70.0
#define HUMIDITY_UNCOMFORTABLE_MIN 70.0 // 70-85%: Uncomfortable
#define HUMIDITY_UNCOMFORTABLE_MAX 85.0
#define HUMIDITY_DANGEROUS_MIN 85.0     // 85+%: Dangerous

// ============================================================================
// INTELLIGENT DATA STRUCTURES
// ============================================================================

struct SensorData {
    float temperature;
    float humidity;
    float motionMagnitude;
    float soundLevel;
    unsigned long timestamp;
};

struct AnalysisResult {
    // Sound Analysis
    String soundStatus;
    bool soundAlert;
    int soundViolationCount;
    
    // Motion Analysis
    String motionStatus;
    bool motionAlert;
    int motionViolationCount;
    
    // Environmental Analysis
    String tempStatus;
    String humidityStatus;
    bool environmentalAlert;
    
    // Overall Health Status
    String overallStatus;
    int alertLevel; // 0=Normal, 1=Warning, 2=Alert, 3=Critical
};

class IntelligentSensorMonitor {
private:
    SensorData dataBuffer[SMOOTHING_SAMPLES];
    int bufferIndex;
    unsigned long lastAnalysis;
    unsigned long lastSample;
    
    // Alert Counters
    int soundAlertCount;
    int motionAlertCount;
    int environmentalAlertCount;
    
    // Trend Analysis
    float tempTrend;
    float humidityTrend;
    float motionTrend;
    float soundTrend;
    
public:
    IntelligentSensorMonitor() {
        bufferIndex = 0;
        lastAnalysis = 0;
        lastSample = 0;
        soundAlertCount = 0;
        motionAlertCount = 0;
        environmentalAlertCount = 0;
        tempTrend = 0;
        humidityTrend = 0;
        motionTrend = 0;
        soundTrend = 0;
        
        // Initialize buffer
        for (int i = 0; i < SMOOTHING_SAMPLES; i++) {
            dataBuffer[i] = {0, 0, 0, 0, 0};
        }
    }
    
    // Add new sensor data with timestamp
    void addData(float temp, float hum, float motion, float sound) {
        unsigned long now = millis();
        
        // Only sample at specified frequency
        if (now - lastSample >= SAMPLE_FREQUENCY_MS) {
            dataBuffer[bufferIndex] = {temp, hum, motion, sound, now};
            bufferIndex = (bufferIndex + 1) % SMOOTHING_SAMPLES;
            lastSample = now;
        }
    }
    
    // Get smoothed data (averaged over multiple samples)
    SensorData getSmoothedData() {
        float tempSum = 0, humSum = 0, motionSum = 0, soundSum = 0;
        int validSamples = 0;
        
        for (int i = 0; i < SMOOTHING_SAMPLES; i++) {
            if (dataBuffer[i].timestamp > 0) {
                tempSum += dataBuffer[i].temperature;
                humSum += dataBuffer[i].humidity;
                motionSum += dataBuffer[i].motionMagnitude;
                soundSum += dataBuffer[i].soundLevel;
                validSamples++;
            }
        }
        
        if (validSamples > 0) {
            return {
                tempSum / validSamples,
                humSum / validSamples,
                motionSum / validSamples,
                soundSum / validSamples,
                millis()
            };
        }
        
        return {0, 0, 0, 0, 0};
    }
    
    // Analyze sound levels intelligently
    String analyzeSound(float soundLevel) {
        if (soundLevel <= SOUND_SILENCE_MAX) return "SILENCE";
        if (soundLevel <= SOUND_LOW_MAX) return "LOW";
        if (soundLevel <= SOUND_MEDIUM_MAX) return "MEDIUM";
        if (soundLevel <= SOUND_HIGH_MAX) return "HIGH";
        return "DANGEROUS";
    }
    
    // Analyze motion intensity intelligently
    String analyzeMotion(float motionMagnitude) {
        if (motionMagnitude <= MOTION_CALM_MAX) return "CALM";
        if (motionMagnitude <= MOTION_NORMAL_MAX) return "NORMAL";
        if (motionMagnitude <= MOTION_ACTIVE_MAX) return "ACTIVE";
        return "VIOLENT";
    }
    
    // Analyze environmental conditions
    String analyzeTemperature(float temp) {
        if (temp >= TEMP_COMFORTABLE_MIN && temp <= TEMP_COMFORTABLE_MAX) return "COMFORTABLE";
        if (temp >= TEMP_UNCOMFORTABLE_MIN && temp <= TEMP_UNCOMFORTABLE_MAX) return "UNCOMFORTABLE";
        return "DANGEROUS";
    }
    
    String analyzeHumidity(float humidity) {
        if (humidity >= HUMIDITY_COMFORTABLE_MIN && humidity <= HUMIDITY_COMFORTABLE_MAX) return "COMFORTABLE";
        if (humidity >= HUMIDITY_UNCOMFORTABLE_MIN && humidity <= HUMIDITY_UNCOMFORTABLE_MAX) return "UNCOMFORTABLE";
        return "DANGEROUS";
    }
    
    // Calculate trends (rate of change)
    void calculateTrends() {
        if (bufferIndex >= 2) {
            int prevIndex = (bufferIndex - 2 + SMOOTHING_SAMPLES) % SMOOTHING_SAMPLES;
            int currIndex = (bufferIndex - 1 + SMOOTHING_SAMPLES) % SMOOTHING_SAMPLES;
            
            if (dataBuffer[prevIndex].timestamp > 0 && dataBuffer[currIndex].timestamp > 0) {
                unsigned long timeDiff = dataBuffer[currIndex].timestamp - dataBuffer[prevIndex].timestamp;
                if (timeDiff > 0) {
                    tempTrend = (dataBuffer[currIndex].temperature - dataBuffer[prevIndex].temperature) / (timeDiff / 1000.0f);
                    humidityTrend = (dataBuffer[currIndex].humidity - dataBuffer[prevIndex].humidity) / (timeDiff / 1000.0f);
                    motionTrend = (dataBuffer[currIndex].motionMagnitude - dataBuffer[prevIndex].motionMagnitude) / (timeDiff / 1000.0f);
                    soundTrend = (dataBuffer[currIndex].soundLevel - dataBuffer[prevIndex].soundLevel) / (timeDiff / 1000.0f);
                }
            }
        }
    }
    
    // Perform comprehensive analysis
    AnalysisResult analyze() {
        AnalysisResult result = {"", false, 0, "", false, 0, "", "", false, "", 0};
        
        unsigned long now = millis();
        
        // Only analyze at specified intervals
        if (now - lastAnalysis >= ANALYSIS_WINDOW_MS) {
            SensorData smoothed = getSmoothedData();
            calculateTrends();
            
            // Sound Analysis
            result.soundStatus = analyzeSound(smoothed.soundLevel);
            if (smoothed.soundLevel > SOUND_HIGH_MAX) {
                soundAlertCount++;
                result.soundAlert = (soundAlertCount >= ALERT_THRESHOLD_COUNT);
                result.soundViolationCount = soundAlertCount;
            } else {
                soundAlertCount = 0;
                result.soundAlert = false;
                result.soundViolationCount = 0;
            }
            
            // Motion Analysis
            result.motionStatus = analyzeMotion(smoothed.motionMagnitude);
            if (smoothed.motionMagnitude > MOTION_ACTIVE_MAX) {
                motionAlertCount++;
                result.motionAlert = (motionAlertCount >= ALERT_THRESHOLD_COUNT);
                result.motionViolationCount = motionAlertCount;
            } else {
                motionAlertCount = 0;
                result.motionAlert = false;
                result.motionViolationCount = 0;
            }
            
            // Environmental Analysis
            result.tempStatus = analyzeTemperature(smoothed.temperature);
            result.humidityStatus = analyzeHumidity(smoothed.humidity);
            result.environmentalAlert = (result.tempStatus == "DANGEROUS" || result.humidityStatus == "DANGEROUS");
            
            // Overall Status Assessment
            int alertScore = 0;
            if (result.soundAlert) alertScore += 1;
            if (result.motionAlert) alertScore += 1;
            if (result.environmentalAlert) alertScore += 2;
            
            if (alertScore == 0) {
                result.overallStatus = "NORMAL";
                result.alertLevel = 0;
            } else if (alertScore == 1) {
                result.overallStatus = "WARNING";
                result.alertLevel = 1;
            } else if (alertScore == 2) {
                result.overallStatus = "ALERT";
                result.alertLevel = 2;
  } else {
                result.overallStatus = "CRITICAL";
                result.alertLevel = 3;
            }
            
            lastAnalysis = now;
        }
        
        return result;
    }
    
    // Get current sensor readings (smoothed)
    SensorData getCurrentReadings() {
        return getSmoothedData();
    }
};

// Global instance
IntelligentSensorMonitor sensorMonitor;

// ============================================================================
// CLEAN, HUMAN-READABLE DISPLAY SYSTEM
// ============================================================================

// Display Settings
#define DISPLAY_INTERVAL_MS 5000        // Show data every 5 seconds
#define AVERAGE_WINDOW_MS 30000         // Average over 30 seconds
#define SIGNIFICANT_CHANGE_TEMP 1.0     // 1°C change
#define SIGNIFICANT_CHANGE_HUM 3.0      // 3% change
#define SIGNIFICANT_CHANGE_MOTION 0.2   // 0.2 m/s² change (gravity-corrected)
#define SIGNIFICANT_CHANGE_SOUND 5      // 5 unit change (calibrated)

class CleanDisplay {
private:
    unsigned long lastDisplay;
    unsigned long lastDataCollection;
    
    // Running averages
    float tempSum, humSum, motionSum, soundSum;
    int sampleCount;
    
    // Previous values for change detection
    float lastTemp, lastHum, lastMotion, lastSound;
    
public:
    CleanDisplay() {
        lastDisplay = 0;
        lastDataCollection = 0;
        tempSum = humSum = motionSum = soundSum = 0;
        sampleCount = 0;
        lastTemp = lastHum = lastMotion = lastSound = -999;
    }
    
    void addData(float temp, float hum, float motion, float sound) {
        unsigned long now = millis();
        
        // Collect data every second
        if (now - lastDataCollection >= 1000) {
            tempSum += temp;
            humSum += hum;
            motionSum += motion;
            soundSum += sound;
            sampleCount++;
            lastDataCollection = now;
        }
    }
    
    void display() {
        unsigned long now = millis();
        
        // Only display every 5 seconds
        if (now - lastDisplay >= DISPLAY_INTERVAL_MS) {
            
            // Calculate averages
            float avgTemp = (sampleCount > 0) ? tempSum / sampleCount : 0;
            float avgHum = (sampleCount > 0) ? humSum / sampleCount : 0;
            float avgMotion = (sampleCount > 0) ? motionSum / sampleCount : 0;
            float avgSound = (sampleCount > 0) ? soundSum / sampleCount : 0;
            
            // Check for significant changes
            bool tempChanged = abs(avgTemp - lastTemp) >= SIGNIFICANT_CHANGE_TEMP;
            bool humChanged = abs(avgHum - lastHum) >= SIGNIFICANT_CHANGE_HUM;
            bool motionChanged = abs(avgMotion - lastMotion) >= SIGNIFICANT_CHANGE_MOTION;
            bool soundChanged = abs(avgSound - lastSound) >= SIGNIFICANT_CHANGE_SOUND;
            
            // Clear screen with separator
            Serial.println();
            Serial.println("╔══════════════════════════════════════════════════════════════╗");
            Serial.println("║                    MENTAL HEALTH MONITOR                    ║");
            Serial.println("║                    Real-Time Sensor Data                    ║");
            Serial.println("╚══════════════════════════════════════════════════════════════╝");
            
            // Display timestamp and calibration info
            Serial.print("🕐 Time: ");
            Serial.print(now / 1000);
            Serial.println(" seconds | Sample Count: " + String(sampleCount));
            Serial.print("🎤 Sound Baseline: ");
            Serial.print(soundCalibrator.getBaseline());
            Serial.println(" units | 📱 Motion: Gravity-corrected");
            Serial.println();
            
            // Display sensor data with clear formatting
            Serial.println("📊 SENSOR READINGS (30-second averages):");
            Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            
            // Temperature
            Serial.print("🌡️  Temperature: ");
            Serial.print(avgTemp, 1);
            Serial.print("°C");
            if (tempChanged) Serial.print(" ⚡ CHANGED");
            Serial.println();
            
            // Humidity
            Serial.print("💧 Humidity:    ");
            Serial.print(avgHum, 1);
            Serial.print("%");
            if (humChanged) Serial.print(" ⚡ CHANGED");
            Serial.println();
            
            // Motion
            Serial.print("📱 Motion:      ");
            Serial.print(avgMotion, 2);
            Serial.print(" m/s²");
            if (motionChanged) Serial.print(" ⚡ CHANGED");
            Serial.println();
            
            // Sound
            Serial.print("🎤 Sound:       ");
            Serial.print(avgSound);
            Serial.print(" units");
            if (soundChanged) Serial.print(" ⚡ CHANGED");
            Serial.println();
            
            Serial.println();
            
            // Display status summary
            Serial.println("📋 STATUS SUMMARY:");
            Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            
            // Temperature status
            if (avgTemp >= 18 && avgTemp <= 26) {
                Serial.println("🌡️  Temperature: ✅ COMFORTABLE");
            } else if (avgTemp > 26 && avgTemp <= 30) {
                Serial.println("🌡️  Temperature: ⚠️  UNCOMFORTABLE");
            } else if (avgTemp > 30) {
                Serial.println("🌡️  Temperature: 🚨 DANGEROUS!");
            } else {
                Serial.println("🌡️  Temperature: ❓ UNKNOWN");
            }
            
            // Humidity status
            if (avgHum >= 30 && avgHum <= 70) {
                Serial.println("💧 Humidity:    ✅ COMFORTABLE");
            } else if (avgHum > 70 && avgHum <= 85) {
                Serial.println("💧 Humidity:    ⚠️  UNCOMFORTABLE");
            } else if (avgHum > 85) {
                Serial.println("💧 Humidity:    🚨 DANGEROUS!");
            } else {
                Serial.println("💧 Humidity:    ❓ UNKNOWN");
            }
            
            // Motion status (gravity-corrected)
            if (avgMotion <= MOTION_CALM_MAX) {
                Serial.println("📱 Motion:      ✅ CALM");
            } else if (avgMotion <= MOTION_NORMAL_MAX) {
                Serial.println("📱 Motion:      ✅ NORMAL");
            } else if (avgMotion <= MOTION_ACTIVE_MAX) {
                Serial.println("📱 Motion:      ⚠️  ACTIVE");
            } else {
                Serial.println("📱 Motion:      🚨 VIOLENT!");
            }
            
            // Sound status (calibrated)
            if (avgSound <= SOUND_SILENCE_MAX) {
                Serial.println("🎤 Sound:       ✅ SILENCE");
            } else if (avgSound <= SOUND_LOW_MAX) {
                Serial.println("🎤 Sound:       ✅ LOW");
            } else if (avgSound <= SOUND_MEDIUM_MAX) {
                Serial.println("🎤 Sound:       ⚠️  MEDIUM");
            } else if (avgSound <= SOUND_HIGH_MAX) {
                Serial.println("🎤 Sound:       ⚠️  HIGH");
            } else {
                Serial.println("🎤 Sound:       🚨 DANGEROUS!");
            }
            
            Serial.println();
            
            // Overall health assessment
            Serial.println("🏥 OVERALL HEALTH ASSESSMENT:");
            Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            
            int alertCount = 0;
            if (avgTemp > 30 || avgHum > 85) alertCount += 2;
            if (avgMotion > MOTION_ACTIVE_MAX) alertCount += 1;
            if (avgSound > SOUND_HIGH_MAX) alertCount += 1;
            
            if (alertCount == 0) {
                Serial.println("✅ ALL SYSTEMS NORMAL - Patient is comfortable");
            } else if (alertCount == 1) {
                Serial.println("⚠️  MINOR WARNING - Some parameters need attention");
            } else if (alertCount == 2) {
                Serial.println("🚨 ALERT - Multiple parameters concerning");
            } else {
                Serial.println("🚨 CRITICAL - Immediate attention required!");
            }
            
            Serial.println();
            Serial.println("╔══════════════════════════════════════════════════════════════╗");
            Serial.println("║                    END OF REPORT                           ║");
            Serial.println("╚══════════════════════════════════════════════════════════════╝");
            Serial.println();
            
            // Reset for next cycle
            tempSum = humSum = motionSum = soundSum = 0;
            sampleCount = 0;
            lastTemp = avgTemp;
            lastHum = avgHum;
            lastMotion = avgMotion;
            lastSound = avgSound;
            lastDisplay = now;
        }
    }
};

// Global clean display instance
CleanDisplay cleanDisplay;

// ============================================================================
// SENSOR INSTANCES
// ============================================================================

HTS221_Direct hts221;
LSM6DS3_Direct lsm6ds3;

// ============================================================================
// MAIN SETUP & LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
    while (!Serial);

    Serial.println("=== MXChip AZ3166 - Direct Hardware Sensor Implementation ===");
    Serial.println("Final Year Project: Mental Health Monitoring System");
    Serial.println("============================================================");
    
    // Initialize sensors with direct hardware access
    Serial.println("Initializing sensors with direct hardware control...");
    
    // First, scan I2C bus to see what devices are present
    Serial.println("Scanning I2C bus...");
    Wire.begin();
    int deviceCount = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("I2C device found at address 0x");
            Serial.println(addr, 16);
            deviceCount++;
        }
    }
    Serial.print("I2C scan complete. Found ");
    Serial.print(deviceCount);
    Serial.println(" devices.");
    
    // Test specific addresses
    Serial.println("Testing specific sensor addresses:");
    Wire.beginTransmission(0x5F); // HTS221
    if (Wire.endTransmission() == 0) {
        Serial.println("✅ HTS221 (0x5F) - RESPONDING");
    } else {
        Serial.println("❌ HTS221 (0x5F) - NOT RESPONDING");
    }
    
    Wire.beginTransmission(0x6A); // LSM6DS3
    if (Wire.endTransmission() == 0) {
        Serial.println("✅ LSM6DS3 (0x6A) - RESPONDING");
    } else {
        Serial.println("❌ LSM6DS3 (0x6A) - NOT RESPONDING");
    }
    
    Wire.beginTransmission(0x6B); // LSM6DS3 alternative
    if (Wire.endTransmission() == 0) {
        Serial.println("✅ LSM6DS3 (0x6B) - RESPONDING");
    } else {
        Serial.println("❌ LSM6DS3 (0x6B) - NOT RESPONDING");
    }
    Serial.println();
    
    bool hts221_ok = hts221.begin();
    bool lsm6ds3_ok = lsm6ds3.begin();
    
    // Calibrate sound sensor
    soundCalibrator.calibrate();
    
    Serial.println("============================================================");
    Serial.println("SENSOR INITIALIZATION SUMMARY:");
    Serial.print("HTS221 (Temperature & Humidity): ");
    Serial.println(hts221_ok ? "✅ OK" : "❌ FAILED");
    Serial.print("LSM6DS3 (Accelerometer & Gyroscope): ");
    Serial.println(lsm6ds3_ok ? "✅ OK" : "❌ FAILED");
    Serial.print("Microphone (Sound Sensor): ");
    Serial.println(soundCalibrator.isReady() ? "✅ CALIBRATED" : "❌ FAILED");
    Serial.println("============================================================");
    
    if (!hts221_ok && !lsm6ds3_ok) {
        Serial.println("ERROR: No sensors working! Check hardware connections.");
    while(1);
  }

    Serial.println("System ready - Reading available sensor data...");
    Serial.println("============================================================");
}

void loop() {
    // Read sensor data
    float temperature = 0.0f, humidity = 0.0f;
    MotionData motion;
    motion.sensorWorking = false; // Default to false
    
    // Read HTS221 (temperature & humidity)
  hts221.readData(temperature, humidity);

    // Read LSM6DS3 (motion) - with fallback
    static bool lsm6ds3_working = true;
    if (lsm6ds3_working) {
        lsm6ds3.readData(motion);
        if (!motion.sensorWorking) {
            lsm6ds3_working = false;
            Serial.println("LSM6DS3: Sensor failed during operation - using fallback");
        }
    } else {
        // Fallback: simulate minimal motion data (gravity-corrected)
        motion.accelX = 0.0f;
        motion.accelY = 0.0f;
        motion.accelZ = 9.81f; // Gravity only
        motion.gyroX = 0.0f;
        motion.gyroY = 0.0f;
        motion.gyroZ = 0.0f;
        motion.motionMagnitude = 0.0f; // No motion (gravity-corrected)
        motion.isMoving = false;
        motion.sensorWorking = false;
    }
    
    // Read microphone (calibrated)
    int micValue = soundCalibrator.getCalibratedSoundLevel();
    
    // Add data to clean display system
    cleanDisplay.addData(temperature, humidity, motion.motionMagnitude, micValue);
    
    // Display clean report every 5 seconds
    cleanDisplay.display();
    
    // Simple delay
    delay(1000);
}
