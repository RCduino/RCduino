/**
 * RCduino Library Implementation
 * Version: 1.0.0
 */

#include "RCduino.h"

// =============================================================================
// RCMotor Implementation
// =============================================================================

RCMotor::RCMotor(uint8_t motorPin, MotorType motorType) {
    pin = motorPin;
    type = motorType;
    reversed = false;
    currentSpeed = 0;
    minSpeed = 0;
    maxSpeed = 100;
}

void RCMotor::begin() {
    if (type == BRUSHLESS_ESC) {
        esc.attach(pin);
        esc.writeMicroseconds(RCDUINO_NEUTRAL_PULSE); // Initialize ESC
        delay(2000); // ESC initialization delay
    } else {
        pinMode(pin, OUTPUT);
    }
}

void RCMotor::setSpeed(int speed) {
    // Constrain speed to limits
    speed = constrain(speed, -maxSpeed, maxSpeed);
    
    if (reversed) speed = -speed;
    
    if (type == BRUSHLESS_ESC) {
        // ESC control (0-100 only)
        int pulseWidth = map(abs(speed), 0, 100, RCDUINO_NEUTRAL_PULSE, RCDUINO_MAX_PULSE_WIDTH);
        esc.writeMicroseconds(pulseWidth);
    } else {
        // DC Motor control
        if (speed > 0) {
            analogWrite(pin, map(speed, 0, 100, 0, 255));
        } else if (speed < 0) {
            analogWrite(pin, map(-speed, 0, 100, 0, 255));
        } else {
            analogWrite(pin, 0);
        }
    }
    
    currentSpeed = speed;
}

void RCMotor::stop() {
    setSpeed(0);
}

void RCMotor::brake() {
    if (type == BRUSHED_DC) {
        digitalWrite(pin, HIGH); // Short brake for DC motors
        delay(50);
        digitalWrite(pin, LOW);
    } else {
        stop();
    }
}

void RCMotor::setReverse(bool reverse) {
    reversed = reverse;
}

void RCMotor::setSpeedLimits(int minSpd, int maxSpd) {
    minSpeed = minSpd;
    maxSpeed = maxSpd;
}

int RCMotor::getSpeed() {
    return currentSpeed;
}

bool RCMotor::isRunning() {
    return currentSpeed != 0;
}

// =============================================================================
// RCServo Implementation
// =============================================================================

RCServo::RCServo(uint8_t servoPin) {
    pin = servoPin;
    currentAngle = 90;
    minAngle = 0;
    maxAngle = 180;
    trimOffset = 0;
    reversed = false;
}

void RCServo::begin() {
    servo.attach(pin);
    center();
}

void RCServo::setAngle(int angle) {
    angle = constrain(angle, minAngle, maxAngle);
    angle += trimOffset;
    
    if (reversed) {
        angle = 180 - angle;
    }
    
    servo.write(angle);
    currentAngle = angle;
}

void RCServo::setMicroseconds(int microseconds) {
    microseconds = constrain(microseconds, RCDUINO_MIN_PULSE_WIDTH, RCDUINO_MAX_PULSE_WIDTH);
    servo.writeMicroseconds(microseconds);
}

void RCServo::setAngleLimits(int minAng, int maxAng) {
    minAngle = minAng;
    maxAngle = maxAng;
}

void RCServo::setTrim(int offset) {
    trimOffset = offset;
}

void RCServo::setReverse(bool reverse) {
    reversed = reverse;
}

void RCServo::center() {
    setAngle(90);
}

int RCServo::getAngle() {
    return currentAngle;
}

void RCServo::detach() {
    servo.detach();
}

// =============================================================================
// RCReceiver Implementation
// =============================================================================

RCReceiver::RCReceiver(SignalType type) {
    signalType = type;
    numChannels = 0;
    signalValid = false;
    lastSignalTime = 0;
    
    for (int i = 0; i < 8; i++) {
        channels[i] = 255;
        channelValues[i] = 0;
    }
}

void RCReceiver::begin() {
    // Initialize pins based on signal type
    for (int i = 0; i < numChannels; i++) {
        if (channels[i] != 255) {
            if (signalType == PWM_SIGNAL) {
                pinMode(channels[i], INPUT);
            } else if (signalType == ANALOG_SIGNAL) {
                pinMode(channels[i], INPUT);
            }
        }
    }
}

void RCReceiver::addChannel(uint8_t channelNum, uint8_t pin) {
    if (channelNum < 8) {
        channels[channelNum] = pin;
        if (channelNum >= numChannels) {
            numChannels = channelNum + 1;
        }
    }
}

void RCReceiver::update() {
    bool validSignal = false;
    
    for (int i = 0; i < numChannels; i++) {
        if (channels[i] != 255) {
            if (signalType == PWM_SIGNAL) {
                int pulseWidth = pulseIn(channels[i], HIGH, 25000); // 25ms timeout
                if (pulseWidth > 0) {
                    channelValues[i] = map(pulseWidth, RCDUINO_MIN_PULSE_WIDTH, RCDUINO_MAX_PULSE_WIDTH, -100, 100);
                    channelValues[i] = constrain(channelValues[i], -100, 100);
                    validSignal = true;
                }
            } else if (signalType == ANALOG_SIGNAL) {
                int analogValue = analogRead(channels[i]);
                channelValues[i] = map(analogValue, 0, 1023, -100, 100);
                validSignal = true;
            }
        }
    }
    
    if (validSignal) {
        lastSignalTime = millis();
        signalValid = true;
    } else if (millis() - lastSignalTime > RCDUINO_DEFAULT_TIMEOUT) {
        signalValid = false;
    }
}

int RCReceiver::getChannelValue(uint8_t channel) {
    if (channel < numChannels && signalValid) {
        return channelValues[channel];
    }
    return 0;
}

int RCReceiver::getChannelRaw(uint8_t channel) {
    if (channel < numChannels && channels[channel] != 255) {
        if (signalType == PWM_SIGNAL) {
            return pulseIn(channels[channel], HIGH, 25000);
        } else if (signalType == ANALOG_SIGNAL) {
            return analogRead(channels[channel]);
        }
    }
    return 0;
}

bool RCReceiver::isSignalValid() {
    return signalValid;
}

unsigned long RCReceiver::getLastSignalTime() {
    return lastSignalTime;
}

void RCReceiver::setChannelReverse(uint8_t channel, bool reverse) {
    // Implementation for channel reverse would go here
}

void RCReceiver::setChannelTrim(uint8_t channel, int trim) {
    // Implementation for channel trim would go here
}

// =============================================================================
// RCBattery Implementation
// =============================================================================

RCBattery::RCBattery(uint8_t pin, float dividerRatio, float refVoltage) {
    analogPin = pin;
    voltageDividerRatio = dividerRatio;
    referenceVoltage = refVoltage;
    currentVoltage = 0.0;
    batteryState = BATTERY_UNKNOWN;
    lastReadTime = 0;
}

void RCBattery::begin() {
    pinMode(analogPin, INPUT);
}

void RCBattery::update() {
    if (millis() - lastReadTime > 1000) { // Update every second
        int analogValue = analogRead(analogPin);
        currentVoltage = (analogValue / 1023.0) * referenceVoltage * voltageDividerRatio;
        
        if (currentVoltage > RCDUINO_LOW_BATTERY_THRESHOLD) {
            batteryState = BATTERY_OK;
        } else if (currentVoltage > RCDUINO_CRITICAL_BATTERY_THRESHOLD) {
            batteryState = BATTERY_LOW;
        } else {
            batteryState = BATTERY_CRITICAL;
        }
        
        lastReadTime = millis();
    }
}

float RCBattery::getVoltage() {
    return currentVoltage;
}

float RCBattery::getVoltagePercentage() {
    // Assuming 3.0V-4.2V range for LiPo
    return map(currentVoltage * 100, 300, 420, 0, 100);
}

BatteryState RCBattery::getBatteryState() {
    return batteryState;
}

bool RCBattery::isLowBattery() {
    return batteryState == BATTERY_LOW || batteryState == BATTERY_CRITICAL;
}

bool RCBattery::isCriticalBattery() {
    return batteryState == BATTERY_CRITICAL;
}

void RCBattery::setThresholds(float lowThreshold, float criticalThreshold) {
    // Custom threshold setting would be implemented here
}

// =============================================================================
// RCSafety Implementation
// =============================================================================

RCSafety::RCSafety() {
    currentState = SAFE;
    signalTimeout = RCDUINO_DEFAULT_TIMEOUT;
    lastSafeTime = 0;
    emergencyStopActive = false;
    battery = nullptr;
    receiver = nullptr;
}

void RCSafety::begin(RCBattery* bat, RCReceiver* rec) {
    battery = bat;
    receiver = rec;
    lastSafeTime = millis();
}

void RCSafety::update() {
    // Check signal loss
    if (receiver && !receiver->isSignalValid()) {
        if (millis() - receiver->getLastSignalTime() > signalTimeout) {
            currentState = SIGNAL_LOST;
        }
    }
    
    // Check battery status
    if (battery) {
        if (battery->isCriticalBattery()) {
            currentState = BATTERY_CRITICAL_WARNING;
        } else if (battery->isLowBattery()) {
            currentState = BATTERY_LOW_WARNING;
        }
    }
    
    // Check emergency stop
    if (emergencyStopActive) {
        currentState = EMERGENCY_STOP;
    }
    
    // Update safe time
    if (currentState == SAFE) {
        lastSafeTime = millis();
    }
}

SafetyState RCSafety::getSafetyState() {
    return currentState;
}

bool RCSafety::isEmergencyStop() {
    return emergencyStopActive;
}

void RCSafety::triggerEmergencyStop() {
    emergencyStopActive = true;
    currentState = EMERGENCY_STOP;
}

void RCSafety::resetEmergencyStop() {
    emergencyStopActive = false;
    currentState = SAFE;
}

void RCSafety::setSignalTimeout(unsigned long timeout) {
    signalTimeout = timeout;
}

bool RCSafety::isSignalLost() {
    return currentState == SIGNAL_LOST;
}

// =============================================================================
// RCStatus Implementation
// =============================================================================

RCStatus::RCStatus(uint8_t led, uint8_t buzzer) {
    ledPin = led;
    buzzerPin = buzzer;
    ledState = false;
    lastBlinkTime = 0;
    blinkPattern = 0;
    buzzerTone = 0;
}

void RCStatus::begin() {
    pinMode(ledPin, OUTPUT);
    if (buzzerPin != 255) {
        pinMode(buzzerPin, OUTPUT);
    }
}

void RCStatus::setLED(bool state) {
    ledState = state;
    digitalWrite(ledPin, state);
}

void RCStatus::blinkLED(int pattern) {
    blinkPattern = pattern;
}

void RCStatus::setBuzzer(int frequency, int duration) {
    if (buzzerPin != 255) {
        tone(buzzerPin, frequency, duration);
    }
}

void RCStatus::playTone(int* notes, int* durations, int noteCount) {
    // Implementation for playing tone sequences
}

void RCStatus::update() {
    // Handle LED blinking
    if (blinkPattern > 0 && millis() - lastBlinkTime > blinkPattern) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastBlinkTime = millis();
    }
}

void RCStatus::indicateStatus(SafetyState state) {
    switch (state) {
        case SAFE:
            setLED(true);
            blinkPattern = 0;
            break;
        case SIGNAL_LOST:
            blinkLED(250); // Fast blink
            setBuzzer(1000, 100);
            break;
        case BATTERY_LOW_WARNING:
            blinkLED(500); // Slow blink
            setBuzzer(800, 200);
            break;
        case BATTERY_CRITICAL_WARNING:
            blinkLED(100); // Very fast blink
            setBuzzer(1200, 300);
            break;
        case EMERGENCY_STOP:
            setLED(false);
            setBuzzer(2000, 1000);
            break;
    }
}

void RCStatus::indicateBattery(BatteryState state) {
    switch (state) {
        case BATTERY_OK:
            setLED(true);
            break;
        case BATTERY_LOW:
            blinkLED(1000);
            break;
        case BATTERY_CRITICAL:
            blinkLED(200);
            setBuzzer(500, 100);
            break;
    }
}

// =============================================================================
// RCduino Main Class Implementation
// =============================================================================

RCduino::RCduino(VehicleType type) {
    vehicleType = type;
    motorCount = 0;
    servoCount = 0;
    initialized = false;
    
    // Initialize pointers
    for (int i = 0; i < 4; i++) {
        motors[i] = nullptr;
        servos[i] = nullptr;
    }
    
    receiver = nullptr;
    battery = nullptr;
    imu = nullptr;
    safety = nullptr;
    status = nullptr;
    gps = nullptr;
    telemetry = nullptr;
}

RCduino::~RCduino() {
    // Cleanup allocated memory
    for (int i = 0; i < 4; i++) {
        if (motors[i]) delete motors[i];
        if (servos[i]) delete servos[i];
    }
    
    if (receiver) delete receiver;
    if (battery) delete battery;
    if (imu) delete imu;
    if (safety) delete safety;
    if (status) delete status;
    if (gps) delete gps;
    if (telemetry) delete telemetry;
}

void RCduino::begin() {
    // Initialize all components
    for (int i = 0; i < motorCount; i++) {
        if (motors[i]) motors[i]->begin();
    }
    
    for (int i = 0; i < servoCount; i++) {
        if (servos[i]) servos[i]->begin();
    }
    
    if (receiver) receiver->begin();
    if (battery) battery->begin();
    if (imu) imu->begin();
    if (status) status->begin();
    if (gps) gps->begin();
    if (telemetry) telemetry->begin();
    
    // Initialize safety system last
    if (safety) safety->begin(battery, receiver);
    
    initialized = true;
}

void RCduino::addMotor(uint8_t pin, MotorType type) {
    if (motorCount < 4) {
        motors[motorCount] = new RCMotor(pin, type);
        motorCount++;
    }
}

void RCduino::addServo(uint8_t pin) {
    if (servoCount < 4) {
        servos[servoCount] = new RCServo(pin);
        servoCount++;
    }
}

void RCduino::addReceiver(SignalType type) {
    if (!receiver) {
        receiver = new RCReceiver(type);
    }
}

void RCduino::addBattery(uint8_t pin, float dividerRatio) {
    if (!battery) {
        battery = new RCBattery(pin, dividerRatio);
    }
}

void RCduino::addIMU() {
    if (!imu) {
        imu = new RCIMU();
    }
}

void RCduino::addGPS() {
    if (!gps) {
        gps = new RCGPS();
    }
}

void RCduino::addTelemetry(long baudRate) {
    if (!telemetry) {
        telemetry = new RCTelemetry();
    }
}

void RCduino::addStatus(uint8_t ledPin, uint8_t buzzerPin) {
    if (!status) {
        status = new RCStatus(ledPin, buzzerPin);
        safety = new RCSafety();
    }
}

void RCduino::update() {
    if (!initialized) return;
    
    // Update all components
    if (receiver) receiver->update();
    if (battery) battery->update();
    if (imu) imu->update();
    if (safety) safety->update();
    if (status) {
        status->update();
        if (safety) {
            status->indicateStatus(safety->getSafetyState());
        }
    }
    if (gps) gps->update();
    if (telemetry) telemetry->update();
    
    // Auto-control based on receiver input (if available and safe)
    if (receiver && receiver->isSignalValid() && safety && safety->getSafetyState() == SAFE) {
        // Basic throttle and steering control
        int throttle = receiver->getChannelValue(0); // Channel 1
        int steering = receiver->getChannelValue(1); // Channel 2
        
        setThrottle(throttle);
        setSteering(steering);
    } else {
        // Safety: stop all motors if signal lost or unsafe
        for (int i = 0; i < motorCount; i++) {
            if (motors[i]) motors[i]->stop();
        }
    }
}

void RCduino::setThrottle(int throttle) {
    if (motorCount > 0 && motors[0]) {
        motors[0]->setSpeed(throttle);
    }
}

void RCduino::setSteering(int steering) {
    if (servoCount > 0 && servos[0]) {
        int angle = map(steering, -100, 100, 45, 135); // Convert to servo angle
        servos[0]->setAngle(angle);
    }
}

void RCduino::setAuxServo(uint8_t servoIndex, int angle) {
    if (servoIndex < servoCount && servos[servoIndex]) {
        servos[servoIndex]->setAngle(angle);
    }
}

void RCduino::emergencyStop() {
    if (safety) {
        safety->triggerEmergencyStop();
    }
    
    // Stop all motors immediately
    for (int i = 0; i < motorCount; i++) {
        if (motors[i]) motors[i]->stop();
    }
}

void RCduino::resetEmergencyStop() {
    if (safety) {
        safety->resetEmergencyStop();
    }
}

SafetyState RCduino::getSafetyState() {
    return safety ? safety->getSafetyState() : SAFE;
}

BatteryState RCduino::getBatteryState() {
    return battery ? battery->getBatteryState() : BATTERY_UNKNOWN;
}

float RCduino::getBatteryVoltage() {
    return battery ? battery->getVoltage() : 0.0;
}

bool RCduino::isReceiverConnected() {
    return receiver ? receiver->isSignalValid() : false;
}

// Component access methods
RCMotor* RCduino::getMotor(uint8_t index) {
    return (index < motorCount) ? motors[index] : nullptr;
}

RCServo* RCduino::getServo(uint8_t index) {
    return (index < servoCount) ? servos[index] : nullptr;
}

RCReceiver* RCduino::getReceiver() {
    return receiver;
}

RCBattery* RCduino::getBattery() {
    return battery;
}

RCIMU* RCduino::getIMU() {
    return imu;
}

RCGPS* RCduino::getGPS() {
    return gps;
}

RCTelemetry* RCduino::getTelemetry() {
    return telemetry;
}

RCStatus* RCduino::getStatus() {
    return status;
}

String RCduino::getVersion() {
    return String(RCDUINO_VERSION);
}

void RCduino::printStatus() {
    Serial.println("=== RCduino Status ===");
    Serial.print("Version: "); Serial.println(getVersion());
    Serial.print("Vehicle Type: "); 
    switch(vehicleType) {
        case RC_CAR: Serial.println("RC Car"); break;
        case RC_PLANE: Serial.println("RC Plane"); break;
        case RC_QUADCOPTER: Serial.println("RC Quadcopter"); break;
        case RC_BOAT: Serial.println("RC Boat"); break;
    }
    
    Serial.print("Motors: "); Serial.println(motorCount);
    Serial.print("Servos: "); Serial.println(servoCount);
    
    if (receiver) {
        Serial.print("Receiver: "); Serial.println(receiver->isSignalValid() ? "Connected" : "Disconnected");
    }
    
    if (battery) {
        Serial.print("Battery: "); Serial.print(battery->getVoltage()); Serial.println("V");
    }
    
    if (safety) {
        Serial.print("Safety State: ");
        switch(safety->getSafetyState()) {
            case SAFE: Serial.println("SAFE"); break;
            case SIGNAL_LOST: Serial.println("SIGNAL LOST"); break;
            case BATTERY_LOW_WARNING: Serial.println("BATTERY LOW"); break;
            case BATTERY_CRITICAL_WARNING: Serial.println("BATTERY CRITICAL"); break;
            case EMERGENCY_STOP: Serial.println("EMERGENCY STOP"); break;
        }
    }
    
    Serial.println("=====================");
}

void RCduino::calibrateIMU() {
    if (imu) {
        Serial.println("Calibrating IMU... Keep vehicle still.");
        imu->calibrate();
        Serial.println("IMU calibration complete.");
    }
}

void RCduino::calibrateReceiver() {
    if (receiver) {
        Serial.println("Receiver calibration not implemented in this version.");
        // Future implementation for receiver calibration
    }
}

// =============================================================================
// RCIMU Implementation (Basic/Placeholder)
// =============================================================================

RCIMU::RCIMU() {
    gyroX = gyroY = gyroZ = 0.0;
    accelX = accelY = accelZ = 0.0;
    pitch = roll = yaw = 0.0;
    initialized = false;
}

bool RCIMU::begin() {
    // In a real implementation, this would initialize I2C and configure the IMU
    // For now, we'll simulate successful initialization
    initialized = true;
    return true;
}

void RCIMU::update() {
    if (!initialized) return;
    
    // Placeholder: In real implementation, read from actual IMU sensor
    // This would involve I2C communication with sensors like MPU6050, etc.
}

void RCIMU::calibrate() {
    if (!initialized) return;
    
    Serial.println("IMU Calibration starting...");
    delay(2000); // Simulate calibration time
    Serial.println("IMU Calibration complete.");
}

float RCIMU::getPitch() { return pitch; }
float RCIMU::getRoll() { return roll; }
float RCIMU::getYaw() { return yaw; }
float RCIMU::getGyroX() { return gyroX; }
float RCIMU::getGyroY() { return gyroY; }
float RCIMU::getGyroZ() { return gyroZ; }
float RCIMU::getAccelX() { return accelX; }
float RCIMU::getAccelY() { return accelY; }
float RCIMU::getAccelZ() { return accelZ; }
bool RCIMU::isCalibrated() { return initialized; }

// =============================================================================
// RCGPS Implementation (Basic/Placeholder)
// =============================================================================

RCGPS::RCGPS() {
    latitude = longitude = altitude = speed = 0.0;
    satellites = 0;
    fixValid = false;
}

bool RCGPS::begin() {
    // Initialize GPS module (usually via Serial)
    return true;
}

void RCGPS::update() {
    // Read GPS data from serial port
    // Parse NMEA sentences
    // Update coordinates, altitude, speed, etc.
}

float RCGPS::getLatitude() { return latitude; }
float RCGPS::getLongitude() { return longitude; }
float RCGPS::getAltitude() { return altitude; }
float RCGPS::getSpeed() { return speed; }
int RCGPS::getSatellites() { return satellites; }
bool RCGPS::hasFix() { return fixValid; }

float RCGPS::distanceTo(float lat, float lon) {
    // Haversine formula implementation
    float dLat = radians(lat - latitude);
    float dLon = radians(lon - longitude);
    float a = sin(dLat/2) * sin(dLat/2) + cos(radians(latitude)) * cos(radians(lat)) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    return 6371000 * c; // Earth radius in meters
}

float RCGPS::bearingTo(float lat, float lon) {
    float dLon = radians(lon - longitude);
    float y = sin(dLon) * cos(radians(lat));
    float x = cos(radians(latitude)) * sin(radians(lat)) - sin(radians(latitude)) * cos(radians(lat)) * cos(dLon);
    return degrees(atan2(y, x));
}

// =============================================================================
// RCTelemetry Implementation
// =============================================================================

RCTelemetry::RCTelemetry() {
    enabled = false;
    lastSendTime = 0;
    sendInterval = 1000; // Default 1 second
}

void RCTelemetry::begin(long baudRate) {
    Serial.begin(baudRate);
    enabled = true;
}

void RCTelemetry::setInterval(int intervalMs) {
    sendInterval = intervalMs;
}

void RCTelemetry::sendTelemetry(float voltage, int speed, float heading, SafetyState safety) {
    if (!enabled) return;
    
    Serial.print("TEL,");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(voltage, 2);
    Serial.print(",");
    Serial.print(speed);
    Serial.print(",");
    Serial.print(heading, 1);
    Serial.print(",");
    Serial.println((int)safety);
}

void RCTelemetry::sendGPS(float lat, float lon, float alt) {
    if (!enabled) return;
    
    Serial.print("GPS,");
    Serial.print(lat, 6);
    Serial.print(",");
    Serial.print(lon, 6);
    Serial.print(",");
    Serial.println(alt, 1);
}

void RCTelemetry::sendIMU(float pitch, float roll, float yaw) {
    if (!enabled) return;
    
    Serial.print("IMU,");
    Serial.print(pitch, 2);
    Serial.print(",");
    Serial.print(roll, 2);
    Serial.print(",");
    Serial.println(yaw, 2);
}

void RCTelemetry::sendCustom(String data) {
    if (!enabled) return;
    
    Serial.print("CUSTOM,");
    Serial.println(data);
}

void RCTelemetry::update() {
    // Periodic telemetry updates could be handled here
}