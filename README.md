
<p align="center">
  <a href="https://www.hirunews.lk/" rel="noopener">
<img src="https://rcduino.github.io/assets/logo.png" alt="RCduino Logo" width="150" height="150" />
</p>
  
  <h1 align="center">RCduino - Arduino RC Vehicle Library</h2>  

## Introduction

RCduino is an advanced Arduino library for building professional-grade RC vehicles including cars, drones, boats and planes.

## Key Features

### 🚗 Motor Control
- Brushed/Brushless motor support
- ESC calibration
- Speed regulation

### 📡 Receiver Input
- PWM/PPM signal decoding
- 6-channel support
- Fail-safe detection

### 🔋 Power Management
- Battery voltage monitoring
- Low voltage cutoff
- Power consumption tracking

### 🛡️ Safety Systems
- Signal loss protection
- Emergency stop
- Overheat prevention

## Quick Start

### Installation
```arduino
#include <RCduino.h>

RCduino myVehicle(RC_CAR);

void setup() {
  myVehicle.begin();
  myVehicle.addMotor(9, BRUSHLESS_ESC);
  myVehicle.addServo(10);
}

void loop() {
  myVehicle.update();
}
```

## Documentation

- [Full API Reference](https://rcduino.github.io/#api-reference)
- [Examples](https://rcduino.github.io/#examples) 
- [Wiring Guides](https://rcduino.github.io/#wiring)

## Supported Boards

| Board        | Notes                          |
|--------------|--------------------------------|
| Arduino Uno  | Full support                   |
| Arduino Mega | Best for complex builds        |
| ESP32        | WiFi/Bluetooth enabled         |

## Community

- [GitHub Issues](https://github.com/RCduino/RCduino/issues)

## License

**MIT License** - Free for personal and commercial use
