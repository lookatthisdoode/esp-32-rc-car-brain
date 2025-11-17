# ESP32 Servo Control System

An advanced servo control system for ESP32 with multiple input modes, designed for precise positioning and gear-like control.

## Features

### Three Control Modes
- **POT Snap**: Potentiometer with discrete positions (0°, 90°, 180°)
- **POT Precise**: Full-range potentiometer control (0-180°)
- **Stick Control**: DualShock 4 controller with D-pad override

### Input Methods
- **Potentiometer**: Analog input on GPIO34 (0-4095 range)
- **DualShock 4**: Bluepad32 wireless controller support
- **D-pad Control**: Discrete gear selection (UP/DOWN)
- **Clutch System**: R2 button for neutral override in D-pad mode

### Display System
- **OLED**: 128x64 SSD1306 display showing real-time values
- **Gear Indicator**: Visual gear display (1, N, 2)
- **Live Bar**: Graphical position indicator with range markers
- **Mode Display**: Current control mode and override status

## Hardware Requirements

- ESP32 Development Board
- Standard Servo Motor (GPIO12)
- 10kΩ Potentiometer (GPIO34)
- SSD1306 OLED Display (I2C)
- DualShock 4 Controller (via Bluetooth)

## Control Schemes

### Potentiometer Modes
- **Snap Mode**: Servo snaps to 0°, 90°, or 180° based on pot ranges (0-60°, 61-120°, 121-180°)
- **Precise Mode**: Direct 1:1 mapping from pot to servo position

### Controller Mode
- **Left Stick**: Full analog control (0-180°)
- **D-pad UP/DOWN**: Step through gears (1→N→2→N→1...)
- **R2 (Clutch)**: Forces neutral position while held (D-pad mode only)
- **Square Button**: Cycles through control modes

### D-pad + Clutch System
1. Use D-pad to select gear
2. Press R2 (clutch) to disengage - servo goes to neutral
3. Change gears with D-pad while clutch held
4. Release clutch - servo jumps to selected gear

## Display Layout

```
POT: 2048 GEAR: N
LX:  -45  LY:   12
R2:  128  DPAD: UP
Mode: D-PAD Control

[====●====] 
1    N    2
```

## Libraries Used

- Adafruit_GFX & Adafruit_SSD1306 (Display)
- ESP32Servo (Servo control)
- Bluepad32 (Controller input)

## Installation

1. Install required libraries via Arduino IDE Library Manager
2. Install Bluepad32 for ESP32
3. Wire hardware according to pin definitions
4. Upload code to ESP32
5. Pair DualShock 4 controller

## Usage

- **Mode Switching**: Press Square button to cycle between modes
- **Gear Selection**: Use D-pad UP/DOWN in controller mode
- **Clutch**: Hold R2 in D-pad mode for neutral override
- **Emergency Reset**: Move left stick to return to analog control

Perfect for robotics projects requiring precise servo positioning with multiple input methods!