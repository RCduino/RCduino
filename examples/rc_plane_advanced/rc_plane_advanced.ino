// File: rc_plane_advanced.ino
// Description: RC plane with ESC, multiple servos, and battery monitoring
// Connections:
// - ESC (Brushless) -> Pin 9
// - Aileron Servo -> Pin 10
// - Elevator Servo -> Pin 11
// - Rudder Servo -> Pin 12
// - PPM Receiver -> Pin 2 (single wire)
// - Battery Monitor -> A0 (with voltage divider)

#include <RCduino.h>

RCduino rc(RC_PLANE);

void setup() {
  // Motor and control surfaces
  rc.addMotor(9, BRUSHLESS_ESC);
  rc.addServo(10);  // Aileron
  rc.addServo(11);  // Elevator
  rc.addServo(12);  // Rudder

  // Receiver setup (PPM)
  rc.addReceiver(PPM_SIGNAL);
  rc.getReceiver()->addChannel(0, 2);  // Throttle
  rc.getReceiver()->addChannel(1, 3);  // Aileron
  rc.getReceiver()->addChannel(2, 4);  // Elevator
  rc.getReceiver()->addChannel(3, 5);  // Rudder

  // Battery monitoring (3S LiPo)
  rc.addBattery(A0, 11.1);

  rc.begin();
  Serial.begin(9600);
  Serial.println("RC Plane Advanced Demo Started");
}

void loop() {
  rc.update();

  // Safety checks
  if (rc.getSafetyState() == BATTERY_LOW_WARNING) {
    Serial.println("WARNING: Low battery! Land soon!");
    rc.setThrottle(50);  // Reduce throttle automatically
  }

  if (rc.getSafetyState() == SIGNAL_LOST) {
    rc.emergencyStop();
    Serial.println("EMERGENCY: Signal lost!");
  }

  // Normal operation
  if (rc.getSafetyState() == SAFE) {
    rc.setThrottle(rc.getReceiver()->getChannelValue(0));
    rc.setAileron(rc.getReceiver()->getChannelValue(1));
    rc.setElevator(rc.getReceiver()->getChannelValue(2));
    rc.setRudder(rc.getReceiver()->getChannelValue(3));
  }
}