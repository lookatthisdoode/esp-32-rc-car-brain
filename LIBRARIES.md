# Included Libraries

This project includes all required libraries in the `libraries/` folder to avoid dependency issues.

## Libraries Included:

- **Adafruit_BusIO** - I2C/SPI communication support
- **Adafruit_GFX_Library** - Graphics primitives for displays
- **Adafruit_SSD1306** - OLED display driver
- **ESP32Servo** - Servo motor control for ESP32
- **PS4Controller** - Alternative controller library (not used)
- **Servo** - Standard Arduino servo library

## Installation

No separate library installation required! All dependencies are bundled.

### Arduino IDE Setup:
1. Copy the entire project folder to your Arduino sketches directory
2. Open `potentiometer-brain.ino` in Arduino IDE
3. The IDE will automatically use the included libraries

### Alternative: Manual Library Installation
If you prefer to use globally installed libraries, you can delete the `libraries/` folder and install via Arduino IDE Library Manager:

- Adafruit GFX Library
- Adafruit SSD1306 
- ESP32Servo
- Bluepad32 (separate installation required)

## Note on Bluepad32

Bluepad32 requires special installation as it's not available in the standard Library Manager. See the main README for installation instructions.