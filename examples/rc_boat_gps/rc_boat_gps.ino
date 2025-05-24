// File: rc_boat_gps.ino
// Description: GPS-assisted autonomous boat with manual override
// Connections:
// - ESC (Brushless) -> Pin 9
// - Rudder Servo -> Pin 10
// - Receiver Ch1 -> Pin 2 (Throttle)
// - Receiver Ch2 -> Pin 3 (Steering)
// - Battery Monitor -> A0
// - GPS Module -> Serial1 (TX1/RX1)

#include <RCduino.h>

RCduino rc(RC_BOAT);

// Waypoint coordinates (example values)
const float TARGET_LAT = 34.0522;
const float TARGET_LON = -118.2437;

void setup() {
  // Drive system
  rc.addMotor(9, BRUSHLESS_ESC);
  rc.addServo(10);  // Rudder

  // Receiver setup
  rc.addReceiver(PWM_SIGNAL);
  rc.getReceiver()->addChannel(0, 2);  // Throttle
  rc.getReceiver()->addChannel(1, 3);  // Steering

  // Sensors
  rc.addBattery(A0, 7.4);  // 2S LiPo
  rc.addGPS(&Serial1);     // Hardware serial for GPS

  rc.begin();
  Serial.begin(9600);
  Serial.println("Autonomous Boat Demo Started");
}

void loop() {
  rc.update();

  // Manual override if signal present
  if (rc.getReceiver()->isSignalValid()) {
    rc.setThrottle(rc.getReceiver()->getChannelValue(0));
    rc.setSteering(rc.getReceiver()->getChannelValue(1));
    return;
  }

  // Autonomous mode when GPS has fix
  if (rc.getGPS()->isFixed()) {
    float currentLat = rc.getGPS()->getLatitude();
    float currentLon = rc.getGPS()->getLongitude();
    
    float distance = calculateDistance(currentLat, currentLon, TARGET_LAT, TARGET_LON);
    int heading = calculateHeading(currentLat, currentLon, TARGET_LAT, TARGET_LON);

    if (distance > 10.0) {  // 10 meter threshold
      rc.setThrottle(70);   // 70% throttle
      rc.setSteering(heading);
    } else {
      rc.setThrottle(0);   // Stop at waypoint
    }
  }
}

// Helper function - calculate distance in meters
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Simplified calculation - replace with Haversine formula
  return sqrt(pow(lat2-lat1, 2) + pow(lon2-lon1, 2)) * 111319.9;
}

// Helper function - calculate heading angle (-100 to 100)
int calculateHeading(float lat1, float lon1, float lat2, float lon2) {
  float y = sin(lon2-lon1) * cos(lat2);
  float x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
  float bearing = atan2(y, x) * (180.0 / PI);
  return map(bearing, -180, 180, -100, 100);
}