/**
 * RCduino Library - Arduino RC Vehicle Control Library
 * Version: 1.0.0
 * Author: RCduino Development Team
 * 
 * A comprehensive library for RC Car and RC Plane control systems
 * Compatible with Arduino Uno, Nano, Mega, and ESP32
 */

#ifndef RCDUINO_H
#define RCDUINO_H

#include <Arduino.h>
#include <Servo.h>

// Library version
#define RCDUINO_VERSION "1.0.0"

// Default configuration constants
#define RCDUINO_DEFAULT_TIMEOUT 2000    // Signal timeout in milliseconds
#define RCDUINO_MIN_PULSE_WIDTH 1000    // Minimum PWM pulse width (µs)
#define RCDUINO_MAX_PULSE_WIDTH 2000    // Maximum PWM pulse width (µs)
#define RCDUINO_NEUTRAL_PULSE 1500      // Neutral PWM pulse width (µs)
#define RCDUINO_LOW_BATTERY_THRESHOLD 3.3 // Low battery voltage threshold
#define RCDUINO_CRITICAL_BATTERY_THRESHOLD 3.0 // Critical battery voltage threshold

// Vehicle types
enum VehicleType {
    RC_CAR,
    RC_PLANE,
    RC_QUADCOPTER,
    RC_BOAT
};

// Motor types
enum MotorType {
    BRUSHED_DC,
    BRUSHLESS_ESC,
    STEPPER
};

// Signal types
enum SignalType {
    PWM_SIGNAL,
    PPM_SIGNAL,
    ANALOG_SIGNAL
};

// Battery states
enum BatteryState {
    BATTERY_OK,
    BATTERY_LOW,
    BATTERY_CRITICAL,
    BATTERY_UNKNOWN
};

// Safety states
enum SafetyState {
    SAFE,
    SIGNAL_LOST,
    BATTERY_LOW_WARNING,
    BATTERY_CRITICAL_WARNING,
    EMERGENCY_STOP
};

/**
 * Motor Control Class
 * Handles DC motors and ESC control with speed regulation
 */
class RCMotor {
private:
    uint8_t pin;
    MotorType type;
    bool reversed;
    int currentSpeed;
    int minSpeed, maxSpeed;
    Servo esc; // For ESC control

public:
    RCMotor(uint8_t motorPin, MotorType motorType = BRUSHED_DC);
    void begin();
    void setSpeed(int speed); // -100 to 100 for DC, 0-100 for ESC
    void stop();
    void brake();
    void setReverse(bool reverse);
    void setSpeedLimits(int minSpd, int maxSpd);
    int getSpeed();
    bool isRunning();
};

/**
 * Servo Control Class
 * Enhanced servo control for steering and flight surfaces
 */
class RCServo {
private:
    Servo servo;
    uint8_t pin;
    int currentAngle;
    int minAngle, maxAngle;
    int trimOffset;
    bool reversed;

public:
    RCServo(uint8_t servoPin);
    void begin();
    void setAngle(int angle); // 0-180 degrees
    void setMicroseconds(int microseconds);
    void setAngleLimits(int minAng, int maxAng);
    void setTrim(int offset);
    void setReverse(bool reverse);
    void center();
    int getAngle();
    void detach();
};

/**
 * Remote Signal Decoder Class
 * Decodes PWM, PPM, and analog signals from RC receivers
 */
class RCReceiver {
private:
    uint8_t channels[8];
    int channelValues[8];
    SignalType signalType;
    unsigned long lastSignalTime;
    bool signalValid;
    uint8_t numChannels;

public:
    RCReceiver(SignalType type = PWM_SIGNAL);
    void begin();
    void addChannel(uint8_t channelNum, uint8_t pin);
    void update();
    int getChannelValue(uint8_t channel); // Returns -100 to 100
    int getChannelRaw(uint8_t channel);   // Returns raw PWM value
    bool isSignalValid();
    unsigned long getLastSignalTime();
    void setChannelReverse(uint8_t channel, bool reverse);
    void setChannelTrim(uint8_t channel, int trim);
};

/**
 * Battery Monitor Class
 * Monitors battery voltage and provides low battery warnings
 */
class RCBattery {
private:
    uint8_t analogPin;
    float voltageDividerRatio;
    float referenceVoltage;
    float currentVoltage;
    BatteryState batteryState;
    unsigned long lastReadTime;
    
public:
    RCBattery(uint8_t pin, float dividerRatio = 2.0, float refVoltage = 5.0);
    void begin();
    void update();
    float getVoltage();
    float getVoltagePercentage();
    BatteryState getBatteryState();
    bool isLowBattery();
    bool isCriticalBattery();
    void setThresholds(float lowThreshold, float criticalThreshold);
};

/**
 * IMU Sensor Class
 * Basic IMU integration for gyroscope and accelerometer
 */
class RCIMU {
private:
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float pitch, roll, yaw;
    bool initialized;

public:
    RCIMU();
    bool begin();
    void update();
    void calibrate();
    float getPitch();
    float getRoll();
    float getYaw();
    float getGyroX();
    float getGyroY();
    float getGyroZ();
    float getAccelX();
    float getAccelY();
    float getAccelZ();
    bool isCalibrated();
};

/**
 * Safety System Class
 * Implements fail-safes and emergency procedures
 */
class RCSafety {
private:
    SafetyState currentState;
    unsigned long signalTimeout;
    unsigned long lastSafeTime;
    bool emergencyStopActive;
    RCBattery* battery;
    RCReceiver* receiver;

public:
    RCSafety();
    void begin(RCBattery* bat = nullptr, RCReceiver* rec = nullptr);
    void update();
    SafetyState getSafetyState();
    bool isEmergencyStop();
    void triggerEmergencyStop();
    void resetEmergencyStop();
    void setSignalTimeout(unsigned long timeout);
    bool isSignalLost();
};

/**
 * Status Indicator Class
 * Controls LEDs and buzzers for status indication
 */
class RCStatus {
private:
    uint8_t ledPin;
    uint8_t buzzerPin;
    bool ledState;
    unsigned long lastBlinkTime;
    int blinkPattern;
    int buzzerTone;

public:
    RCStatus(uint8_t led = 13, uint8_t buzzer = 255);
    void begin();
    void setLED(bool state);
    void blinkLED(int pattern = 500); // Blink interval in ms
    void setBuzzer(int frequency, int duration = 100);
    void playTone(int* notes, int* durations, int noteCount);
    void update();
    void indicateStatus(SafetyState state);
    void indicateBattery(BatteryState state);
};

/**
 * GPS Interface Class (Optional)
 * Basic GPS coordinate and navigation support
 */
class RCGPS {
private:
    float latitude, longitude;
    float altitude;
    float speed;
    int satellites;
    bool fixValid;

public:
    RCGPS();
    bool begin();
    void update();
    float getLatitude();
    float getLongitude();
    float getAltitude();
    float getSpeed();
    int getSatellites();
    bool hasFix();
    float distanceTo(float lat, float lon);
    float bearingTo(float lat, float lon);
};

/**
 * Telemetry Class
 * Handles telemetry data transmission via Serial
 */
class RCTelemetry {
private:
    bool enabled;
    unsigned long lastSendTime;
    int sendInterval;

public:
    RCTelemetry();
    void begin(long baudRate = 9600);
    void setInterval(int intervalMs);
    void sendTelemetry(float voltage, int speed, float heading, SafetyState safety);
    void sendGPS(float lat, float lon, float alt);
    void sendIMU(float pitch, float roll, float yaw);
    void sendCustom(String data);
    void update();
};

/**
 * Main RCduino Class
 * Integrates all components for easy vehicle control
 */
class RCduino {
private:
    VehicleType vehicleType;
    RCMotor* motors[4];
    RCServo* servos[4];
    RCReceiver* receiver;
    RCBattery* battery;
    RCIMU* imu;
    RCSafety* safety;
    RCStatus* status;
    RCGPS* gps;
    RCTelemetry* telemetry;
    uint8_t motorCount, servoCount;
    bool initialized;

public:
    RCduino(VehicleType type = RC_CAR);
    ~RCduino();
    
    // Initialization
    void begin();
    void addMotor(uint8_t pin, MotorType type = BRUSHED_DC);
    void addServo(uint8_t pin);
    void addReceiver(SignalType type = PWM_SIGNAL);
    void addBattery(uint8_t pin, float dividerRatio = 2.0);
    void addIMU();
    void addGPS();
    void addTelemetry(long baudRate = 9600);
    void addStatus(uint8_t ledPin = 13, uint8_t buzzerPin = 255);
    
    // Main control loop
    void update();
    
    // Vehicle control methods
    void setThrottle(int throttle); // -100 to 100
    void setSteering(int steering); // -100 to 100
    void setAuxServo(uint8_t servoIndex, int angle);
    void emergencyStop();
    void resetEmergencyStop();
    
    // Status methods
    SafetyState getSafetyState();
    BatteryState getBatteryState();
    float getBatteryVoltage();
    bool isReceiverConnected();
    
    // Component access
    RCMotor* getMotor(uint8_t index);
    RCServo* getServo(uint8_t index);
    RCReceiver* getReceiver();
    RCBattery* getBattery();
    RCIMU* getIMU();
    RCGPS* getGPS();
    RCTelemetry* getTelemetry();
    RCStatus* getStatus();
    
    // Utility methods
    String getVersion();
    void printStatus();
    void calibrateIMU();
    void calibrateReceiver();
};

#endif // RCDUINO_H