#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pin definitions
#define POT_PIN 34        // Potentiometer pin (GPIO34 - analog input)
#define MOTOR_PIN 13      // Motor ESC pin
#define SERVO_PIN 12      // Servo pin

// Motor ESC and Servo
Servo motorESC;
Servo testServo;

// Controller pointer
ControllerPtr dualshock;

// Control mode enum
enum ControlMode {
  POT_SNAP = 0,
  POT_PRECISE = 1,
  STICK_CONTROL = 2
};

// Control mode switch
ControlMode controlMode = POT_SNAP;
bool lastXState = false;     // Track X button state for toggle

// Position tracking
int currentPosition = 0;     // Current position (1, 2, or 3)
int lastServoAngle = -1;     // Track last servo angle to avoid repeated writes

// D-pad gear override
bool dpadOverride = false;   // When true, use dpad gear instead of stick
int dpadGear = 2;           // Current D-pad selected gear (1, 2, 3)
bool lastUpState = false;    // Track UP D-pad state
bool lastDownState = false;  // Track DOWN D-pad state
bool clutchEngaged = false;  // When true, force to N position (clutch)

void onConnectedController(ControllerPtr ctl) {
  Serial.println("Controller connected!");
  dualshock = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println("Controller disconnected!");
  if (ctl == dualshock) {
    dualshock = nullptr;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Potentiometer Motor Control");

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }

  // Initialize motor ESC (commented out)
  //motorESC.attach(MOTOR_PIN, 1000, 2000); // (pin, min pulse width, max pulse width)
  
  // Initialize servo on pin 12
  testServo.attach(SERVO_PIN);
  
  /* Servo library usage notes:
   * servo.attach(pin, 1000, 2000) - Sets pulse width range (min, max)
   * servo.write(0-180)           - Angle mode: 0=1000μs, 90=1500μs, 180=2000μs
   * servo.writeMicroseconds(1000-2000) - Direct pulse width control
   * 
   * ESC: 0=STOP, 90=HALF_SPEED, 180=FULL_SPEED
   * Regular servo: 0=0°, 90=90°, 180=180°
   * 
   * Example:
   * motorESC.attach(MOTOR_PIN, 1000, 2000);  // Sets pulse range
   * motorESC.write(0-180);                   // Maps to 1000-2000μs
   */

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Pot + Controller");
  display.setCursor(0,10);
  display.println("Waiting for DS4...");
  display.display();
  
  // Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
}

void loop() {
  BP32.update();
  
  // Read potentiometer value
  int potValue = analogRead(POT_PIN);   // 0-4095
  int servoAngle = map(potValue, 0, 4095, 0, 180);   // Map to servo range 0-180
  
  // Determine position based on servo angle ranges
  int newPosition;
  if (servoAngle <= 60) {
    newPosition = 1;
  } else if (servoAngle <= 120) {
    newPosition = 2;
  } else {
    newPosition = 3;
  }
  
  // Only update servo if position changed
  if (newPosition != currentPosition) {
    currentPosition = newPosition;
    // Map position to actual servo angles
    int targetAngle;
    if (currentPosition == 1) targetAngle = 0;
    else if (currentPosition == 2) targetAngle = 90;
    else targetAngle = 180;
    
    testServo.write(targetAngle);
    lastServoAngle = targetAngle;
    Serial.printf("Position changed to %d (servo: %d degrees)\n", currentPosition, targetAngle);
  }

  // Display on OLED
  display.clearDisplay();
  
  if (dualshock && dualshock->isConnected()) {
    // Read controller values
    int lx = dualshock->axisX();
    int ly = dualshock->axisY();
    int r2 = dualshock->throttle();
    
    // Button readings for future use
    bool cross = dualshock->a();
    bool circle = dualshock->b();
    bool square = dualshock->x();
    bool triangle = dualshock->y();
    uint8_t dpadValue = dualshock->dpad();
    bool xPressed = square;  // Using square button for toggle
    
    // Toggle control mode on X button press (edge detection)
    if (xPressed && !lastXState) {
      controlMode = (ControlMode)((controlMode + 1) % 3);
      if (controlMode == POT_SNAP) Serial.println("Switched to POT snap");
      else if (controlMode == POT_PRECISE) Serial.println("Switched to POT precise");
      else Serial.println("Switched to LX control");
    }
    lastXState = xPressed;
    
    // D-pad gear control (only in controller mode)
    if (controlMode == STICK_CONTROL) {
      // Initialize dpadGear to current stick position if not overridden
      if (!dpadOverride) {
        int currentGear = (lx < -170) ? 1 : (lx > 170) ? 3 : 2;
        dpadGear = currentGear;
      }
      
      // UP D-pad: increase gear
      if (dpadValue == 1 && !lastUpState) {
        if (dpadGear < 3) dpadGear++;
        dpadOverride = true;
        // Write to servo immediately (unless clutch is engaged)
        if (!clutchEngaged) {
          int targetAngle = (dpadGear == 1) ? 0 : (dpadGear == 2) ? 90 : 180;
          testServo.write(targetAngle);
        }
        Serial.printf("D-pad UP gear: %d\n", dpadGear);
      }
      
      // DOWN D-pad: decrease gear
      if (dpadValue == 2 && !lastDownState) {
        if (dpadGear > 1) dpadGear--;
        dpadOverride = true;
        // Write to servo immediately (unless clutch is engaged)
        if (!clutchEngaged) {
          int targetAngle = (dpadGear == 1) ? 0 : (dpadGear == 2) ? 90 : 180;
          testServo.write(targetAngle);
        }
        Serial.printf("D-pad DOWN gear: %d\n", dpadGear);
      }
      
      // R2 clutch for N position (only works in D-pad mode)
      if (dpadOverride) {
        bool clutchPressed = (r2 > 100);  // R2 is analog, check if pressed significantly
        if (clutchPressed && !clutchEngaged) {
          clutchEngaged = true;
          testServo.write(90);  // Force to N position (clutch engaged)
          Serial.println("Clutch engaged - forced to N position");
        } else if (!clutchPressed && clutchEngaged) {
          clutchEngaged = false;
          // Return to current D-pad gear
          int targetAngle = (dpadGear == 1) ? 0 : (dpadGear == 2) ? 90 : 180;
          testServo.write(targetAngle);
          Serial.printf("Clutch disengaged - returning to D-pad gear %d\n", dpadGear);
        }
      } else {
        clutchEngaged = false;  // Reset clutch when not in D-pad mode
      }
      
      // Exit D-pad override when stick moves significantly
      if (dpadOverride && abs(lx) > 50) {
        dpadOverride = false;
        clutchEngaged = false;  // Reset clutch when exiting D-pad mode
        Serial.println("Stick moved - returning to stick control");
      }
    } else {
      dpadOverride = false;
      clutchEngaged = false;
    }
    
    // Update state tracking for D-pad
    lastUpState = (dpadValue == 1);     // Track UP state
    lastDownState = (dpadValue == 2);   // Track DOWN state
    
    // Choose servo angle based on mode
    if (controlMode == STICK_CONTROL && !dpadOverride && !clutchEngaged) {
      servoAngle = map(lx, -512, 511, 0, 180);
      servoAngle = constrain(servoAngle, 0, 180);
      testServo.write(servoAngle);
    } else if (controlMode == POT_PRECISE) {
      // POT precise mode - map directly without snapping
      servoAngle = map(potValue, 0, 4095, 0, 180);
      testServo.write(servoAngle);
    }
    
    // Display controller + pot values
    display.setTextSize(1);
    display.setCursor(0,0);
    int displayGear;
    if (controlMode == STICK_CONTROL) {
      // In controller mode, determine gear based on overrides or LX position
      if (clutchEngaged) {
        displayGear = 2;  // Clutch forces N position
      } else if (dpadOverride) {
        displayGear = dpadGear;
      } else {
        displayGear = (lx < -170) ? 1 : (lx > 170) ? 3 : 2;
      }
    } else {
      // In pot modes, determine gear based on servo angle
      if (servoAngle <= 60) displayGear = 1;
      else if (servoAngle <= 120) displayGear = 2;
      else displayGear = 3;
    }
    char gearChar = (displayGear == 1) ? '1' : (displayGear == 2) ? 'N' : '2';
    display.printf("POT: %4d GEAR: %c", potValue, gearChar);
    display.setCursor(0,10);
    display.printf("LX: %4d  LY: %4d", lx, ly);
    display.setCursor(0,20);
    // Convert D-pad value to direction string (corrected mapping)
    String dpadDir = "NONE";
    if (dpadValue == 1) dpadDir = "UP";      // UP is correct
    else if (dpadValue == 2) dpadDir = "DOWN";    // DOWN was showing as UR, so down=2
    else if (dpadValue == 4) dpadDir = "RIGHT";   // RIGHT was showing as DR, so right=4  
    else if (dpadValue == 8) dpadDir = "LEFT";    // LEFT was showing NONE, try 8
    else dpadDir = String(dpadValue);             // Show raw value for debugging
    display.printf("R2: %4d  DPAD: %s", r2, dpadDir.c_str());
    display.setCursor(0,30);
    if (controlMode == POT_SNAP) display.println("Mode: POT snap");
    else if (controlMode == POT_PRECISE) display.println("Mode: POT precise"); 
    else if (clutchEngaged) display.println("Clutch engaged");
    else if (dpadOverride) display.println("Mode: D-PAD");
    else display.println("Mode: Left Stick");
    
    // Draw live control bar
    int barWidth = 108;  // Bar width in pixels
    int barHeight = 8;   // Bar height
    int barX = 10;       // Bar X position
    int barY = 45;       // Bar Y position
    
    // Draw outer frame
    display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
    
    // Calculate indicator position based on control mode
    int indicatorPos;
    if (controlMode == STICK_CONTROL) {
      if (clutchEngaged) {
        // Clutch engaged - show N position (center)
        indicatorPos = barWidth/2 - 2;
      } else if (dpadOverride) {
        // Show D-pad gear position
        if (dpadGear == 1) indicatorPos = 2;
        else if (dpadGear == 2) indicatorPos = barWidth/2 - 2;
        else indicatorPos = barWidth - 6;
      } else {
        // Map controller LX (-512 to 511) to bar position
        indicatorPos = map(lx, -512, 511, 2, barWidth-6);
      }
    } else {
      // Map pot value (0-4095) to bar position
      indicatorPos = map(potValue, 0, 4095, 2, barWidth-6);
    }
    
    // Draw position indicator (4px wide)
    display.fillRect(barX + indicatorPos, barY + 2, 4, barHeight - 4, SSD1306_WHITE);
    
    // Draw position markers for gear positions (only in pot modes)
    if (controlMode != STICK_CONTROL) {
      int pos1 = map(60 * 4095 / 180, 0, 4095, 2, barWidth-6);
      int pos2 = map(120 * 4095 / 180, 0, 4095, 2, barWidth-6);
      display.drawFastVLine(barX + pos1, barY, barHeight, SSD1306_WHITE);
      display.drawFastVLine(barX + pos2, barY, barHeight, SSD1306_WHITE);
    }
    
    // Add gear labels under the bar
    display.setTextSize(1);
    display.setCursor(barX, barY + barHeight + 2);
    display.print("1");
    display.setCursor(barX + barWidth/2 - 2, barY + barHeight + 2);
    display.print("N");
    display.setCursor(barX + barWidth - 6, barY + barHeight + 2);
    display.print("2");
    
  } else {
    // No controller - show pot only
    display.setTextSize(2);
    display.setCursor(0,0);
    display.printf("POT: %4d", potValue);
    
    display.setCursor(0,20);
    char gearChar = (currentPosition == 1) ? '1' : (currentPosition == 2) ? 'N' : '2';
    display.printf("GEAR: %c", gearChar);
    
    display.setCursor(0,40);
    display.setTextSize(1);
    display.println("Bluetooth ON");
  }
  
  display.display();
  delay(50);
}