// File: rc_car_basic.ino
// Description: Basic RC car with throttle & steering control
// Connections:
// - Motor (Brushed DC) -> Pin 9
// - Steering Servo -> Pin 10
// - Receiver Ch1 (Throttle) -> Pin 2
// - Receiver Ch2 (Steering) -> Pin 3

#include <RCduino.h>

RCduino rc(RC_CAR);

void setup() {
  // Hardware setup
  rc.addMotor(9, BRUSHED_DC);
  rc.addServo(10);
  
  // Receiver setup (PWM)
  rc.addReceiver(PWM_SIGNAL);
  rc.getReceiver()->addChannel(0, 2);  // Throttle
  rc.getReceiver()->addChannel(1, 3);  // Steering

  rc.begin();
  Serial.begin(9600);
  Serial.println("RC Car Basic Demo Started");
}

void loop() {
  rc.update();  // Required for all RCduino operations

  // Read and apply controls
  int throttle = rc.getReceiver()->getChannelValue(0);
  int steering = rc.getReceiver()->getChannelValue(1);
  
  rc.setThrottle(throttle);
  rc.setSteering(steering);

  // Debug output
  Serial.print("Throttle: ");
  Serial.print(throttle);
  Serial.print(" | Steering: ");
  Serial.println(steering);
  
  delay(20);  // Small delay to prevent serial flooding
}