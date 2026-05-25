#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "splash.h"
// ─── Pins ────────────────────────────────────────────────────────────────────
#define MOTOR_PIN 19
#define SERVO_PIN 18
#define SHIFTER_PIN 23
#define OLED_SDA 21
#define OLED_SCL 22
#define ENC_CLK 15
#define ENC_DT 4
#define ENC_BTN 5

struct Config {
  int maxTiming;
  int deadzone;
  float expAccelRate;
  float expDecayRate;
  int servoCenter;
  int servoMin;
  int servoMax;
  bool servoReverse;
};

// ─── Presets ─────────────────────────────────────────────────────────────────
const char* presetNames[] = { "Slow", "Normal", "Fast" };
const Config presets[] = {
  { 1200, 15, 0.15f, 0.05f, 90, 0, 180, false },  // Slow
  { 1400, 10, 0.30f, 0.09f, 90, 0, 180, false },  // Normal
  { 1800,  5, 0.50f, 0.15f, 90, 0, 180, false },  // Fast
};
const int presetCount = 3;
int currentPreset = 1;
Config cfg = presets[1];

bool lastL1 = false, lastR1 = false;
unsigned long presetAlertTime = 0;
#define PRESET_ALERT_MS 1500

// ─── OLED ────────────────────────────────────────────────────────────────────
Adafruit_SSD1306 display(128, 64, &Wire, -1);
const bool hasSplash = true;

// ─── Servo / ESC ─────────────────────────────────────────────────────────────
Servo motorESC;
Servo steeringServo;
Servo shifterServo;

// ─── Bluepad32 ───────────────────────────────────────────────────────────────
ControllerPtr dualshock;
bool ctrlConnected = false;
int lxVal = 0, ryVal = 0, r2Val = 0, l2Val = 0;

void onConnectedController(ControllerPtr ctl) {
  Serial.println(">>> Controller connected!");
  dualshock = ctl;
  ctrlConnected = true;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println(">>> Controller disconnected!");
  if (ctl == dualshock) dualshock = nullptr;
  ctrlConnected = false;
  lxVal = ryVal = r2Val = l2Val = 0;
}

// ─── Shifter ─────────────────────────────────────────────────────────────────
enum Gear { GEAR_D, GEAR_N, GEAR_R };
Gear currentGear = GEAR_N;
bool lastDpadUp = false, lastDpadDown = false;

int gearToAngle(int g) {
  switch (g) {
    case GEAR_D: return 0;
    case GEAR_N: return 90;
    case GEAR_R: return 180;
  }
  return 90;
}

char gearChar(int g) {
  switch (g) {
    case GEAR_D: return 'D';
    case GEAR_N: return 'N';
    case GEAR_R: return 'R';
  }
  return '?';
}

// ─── Global state ────────────────────────────────────────────────────────────
int motorSignal = 1000;
int servoAngle = cfg.servoCenter;
int shifterAngle = 90;
bool expMode = false;
bool handbrake = false;
bool lastStartState = false;
float smoothSignal = 1000;
int   manualMotor     = 1000;
int   manualServo     = 90;
int   manualShifter   = 90;
int   manualEditTarget = 0; // 0=motor, 1=steering, 2=shifter
unsigned long lastActivityTime = 0;
#define MENU_TIMEOUT_MS 10000

// ─── Encoder  ─────────────────────────────────
uint8_t stableRead(int pin) {
  uint8_t count = 0;
  for (int i = 0; i < 3; i++) {
    count += digitalRead(pin);
    delayMicroseconds(50);
  }
  return (count >= 2) ? 1 : 0;
}

int readEncoder() {
  static uint8_t prevState = 3;
  static int     accum     = 0;
  static unsigned long lastTick = 0;
  static unsigned long lastChange = 0;

  uint8_t clk   = stableRead(ENC_CLK);
  uint8_t dt    = stableRead(ENC_DT);
  uint8_t state = (clk << 1) | dt;

  if (state != prevState && millis() - lastChange >= 2) {
    if (prevState == 3 && state == 1) accum++;
    if (prevState == 1 && state == 0) accum++;
    if (prevState == 3 && state == 2) accum--;
    if (prevState == 2 && state == 0) accum--;
    prevState = state;
    lastChange = millis();
    lastTick = millis();
  }

  int delta = 0;
  unsigned long age = millis() - lastTick;
  int threshold = (age < 80) ? 1 : 2; // 2:4 for less noise but less responsive

  if (accum >=  threshold) { delta =  1; accum = 0; }
  if (accum <= -threshold) { delta = -1; accum = 0; }
  return delta;
}

// ─── Button — debounce + long press ──────────────────────────────────────────
#define LONG_PRESS_MS 800
unsigned long btnPressTime = 0;
bool btnWasPressed = false;
bool longPressHandled = false;

// returns 1 = short click, 2 = long press, 0 = nothing
int readButton() {
  bool pressed = (digitalRead(ENC_BTN) == LOW);
  int result = 0;
  if (pressed && !btnWasPressed) {
    btnPressTime = millis();
    btnWasPressed = true;
    longPressHandled = false;
  }
  if (pressed && btnWasPressed && !longPressHandled) {
    if (millis() - btnPressTime >= LONG_PRESS_MS) {
      longPressHandled = true;
      result = 2;
    }
  }
  if (!pressed && btnWasPressed) {
    btnWasPressed = false;
    if (!longPressHandled && millis() - btnPressTime < LONG_PRESS_MS)
      result = 1;
  }
  return result;
}

// ─── Menu ────────────────────────────────────────────────────────────────────
enum Screen {
  SCR_DRIVE,
  SCR_MAIN_MENU,
  SCR_MOTOR_MENU,
  SCR_SERVO_MENU,
  SCR_PRESET_MENU,
  SCR_CTRL_MONITOR,
  SCR_MANUAL_CONTROL,
  SCR_EDIT_VALUE
};

Screen currentScreen = SCR_DRIVE;

const char* mainMenuItems[] = { "Motor", "Servo", "Mode", "Controller", "Manual Control" };
const int mainMenuCount = 5;
const char* motorMenuItems[] = { "Max timing", "Deadzone", "Accel rate", "Decay rate" };
const int motorMenuCount = 4;
const char* servoMenuItems[] = { "Servo min", "Servo max", "Reverse", "Center" };
const int servoMenuCount = 4;

int menuIndex = 0;
Screen prevMenu = SCR_MAIN_MENU;
int editingItem = 0;

enum EditType { EDIT_INT,
                EDIT_FLOAT,
                EDIT_BOOL };

struct EditState {
  const char* label;
  EditType type;
  int intVal, intMin, intMax, intStep;
  float floatVal, floatMin, floatMax, floatStep;
  bool boolVal;
} edit;

void enterEditInt(const char* label, int val, int mn, int mx, int step) {
  edit = { label, EDIT_INT, val, mn, mx, step, 0, 0, 0, 0, false };
  prevMenu = currentScreen;
  currentScreen = SCR_EDIT_VALUE;
}

void enterEditFloat(const char* label, float val, float mn, float mx, float step) {
  edit = { label, EDIT_FLOAT, 0, 0, 0, 0, val, mn, mx, step, false };
  prevMenu = currentScreen;
  currentScreen = SCR_EDIT_VALUE;
}

void enterEditBool(const char* label, bool val) {
  edit = { label, EDIT_BOOL, 0, 0, 0, 0, 0, 0, 0, 0, val };
  prevMenu = currentScreen;
  currentScreen = SCR_EDIT_VALUE;
}

void applyEdit() {
  if (prevMenu == SCR_MOTOR_MENU) {
    switch (editingItem) {
      case 0: cfg.maxTiming = edit.intVal; break;
      case 1: cfg.deadzone = edit.intVal; break;
      case 2: cfg.expAccelRate = edit.floatVal; break;
      case 3: cfg.expDecayRate = edit.floatVal; break;
    }
  } else if (prevMenu == SCR_SERVO_MENU) {
    switch (editingItem) {
      case 0: cfg.servoMin = edit.intVal; break;
      case 1: cfg.servoMax = edit.intVal; break;
      case 2: cfg.servoReverse = edit.boolVal; break;
      case 3: cfg.servoCenter = edit.intVal; break;
    }
  }
}

// ─── Display ─────────────────────────────────────────────────────────────────

void drawCarIcon(int x, int y) {
  // top-down car, ~12x20px
  display.fillRect(x + 3, y, 6, 20, SSD1306_WHITE);       // body
  display.fillRect(x, y + 2, 3, 5, SSD1306_WHITE);         // front-left wheel
  display.fillRect(x + 9, y + 2, 3, 5, SSD1306_WHITE);     // front-right wheel
  display.fillRect(x, y + 13, 3, 5, SSD1306_WHITE);        // rear-left wheel
  display.fillRect(x + 9, y + 13, 3, 5, SSD1306_WHITE);    // rear-right wheel
  display.fillRect(x + 4, y + 4, 4, 3, SSD1306_BLACK);     // windshield
  display.fillRect(x + 4, y + 12, 4, 3, SSD1306_BLACK);    // rear window
}

void drawMotorBar(int x, int y, int w, int h, int signal) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  float pct = (signal - 1000) / (float)(cfg.maxTiming - 1000);
  int fill = constrain((int)(pct * (w - 4)), 0, w - 4);
  if (fill > 0) display.fillRect(x + 2, y + 2, fill, h - 4, SSD1306_WHITE);
}

void drawSteerBar(int x, int y, int w, int h, int angle) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int blobW = 5;
  int pos = cfg.servoReverse
              ? map(angle, cfg.servoMax, cfg.servoMin, x + 2, x + w - 2 - blobW)
              : map(angle, cfg.servoMin, cfg.servoMax, x + 2, x + w - 2 - blobW);
  pos = constrain(pos, x + 2, x + w - 2 - blobW);
  display.fillRect(pos, y + 1, blobW, h - 2, SSD1306_WHITE);
}

void drawScrollMenu(const char* title, const char** items, int count, int selected) {
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 1);
  display.print(title);
  display.setTextColor(SSD1306_WHITE);
  int start = max(0, min(selected, count - 4));
  for (int i = start; i < min(start + 4, count); i++) {
    int y = 12 + (i - start) * 13;
    if (i == selected) {
      display.fillRect(0, y - 1, 128, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(4, y);
    display.print(items[i]);
    display.setTextColor(SSD1306_WHITE);
  }
  display.display();
}

void drawEditScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 1);
  display.print(edit.label);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 22);
  if (edit.type == EDIT_INT) display.printf("%d", edit.intVal);
  else if (edit.type == EDIT_FLOAT) display.printf("%.2f", edit.floatVal);
  else display.print(edit.boolVal ? "YES" : "NO");
  display.setTextSize(1);
  display.setCursor(2, 54);
  display.print("click=save  long=back");
  display.display();
}

void updateDriveDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  if (ctrlConnected) {
    display.setCursor(0, 0);
    display.printf("%s%s [%c]%s", presetNames[currentPreset], expMode ? " [EXP]" : "", gearChar(currentGear), handbrake ? " BRK" : "");
    display.setCursor(0, 10);
    display.printf("Motor: %4dus", motorSignal);
    drawMotorBar(0, 19, 128, 8, motorSignal);
    display.setCursor(0, 30);
    display.printf("Steer: %3ddeg", servoAngle);
    drawSteerBar(0, 39, 128, 8, servoAngle);
    display.setCursor(0, 50);
    display.printf("R2:%4d L2:%4d LX:%4d", r2Val, l2Val, lxVal);
  } else {
    // drawCarIcon(8, 22);
    display.setTextSize(1);
    display.setCursor(30, 8);
    display.print("RC Car Ready");
    display.setCursor(30, 22);
    display.print("Waiting for");
    display.setCursor(30, 32);
    display.print("controller...");
    display.drawLine(0, 50, 128, 50, SSD1306_WHITE);
    display.setCursor(4, 54);
    display.print("Click once for menu");
  }

  if (millis() - presetAlertTime < PRESET_ALERT_MS) {
    display.fillRect(14, 18, 100, 28, SSD1306_BLACK);
    display.drawRect(14, 18, 100, 28, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(20, 24);
    display.printf("<%s>", presetNames[currentPreset]);
    display.setTextSize(1);
  }

  display.display();
}

void updateCtrlMonitor() {
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 1);
  display.print("Controller");
  display.setTextColor(SSD1306_WHITE);
  if (ctrlConnected) {
    display.setCursor(0, 13);
    display.printf("LX:%5d  RY:%5d", lxVal, ryVal);
    display.setCursor(0, 24);
    display.printf("R2:%5d  L2:%5d", r2Val, l2Val);
    display.setCursor(0, 35);
    display.printf("EXP:%-3s  Gear:[%c]", expMode ? "ON" : "OFF", gearChar(currentGear));
    display.setCursor(0, 46);
    display.printf("Brake: %s", handbrake ? "ON" : "OFF");
  } else {
    display.setCursor(4, 28);
    display.print("Not connected");
  }
  display.setCursor(0, 56);
  display.print("long press = back");
  display.display();
}

void updateManualControl() {
  const char* targetNames[] = { "MOTOR", "STEER", "SHIFT" };
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 1);
  display.printf("Manual [%s]", targetNames[manualEditTarget]);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 13);
  display.printf("Motor: %4d", manualMotor);
  if (manualEditTarget == 0) display.print(" <");
  drawMotorBar(0, 22, 128, 8, manualMotor);

  display.setCursor(0, 33);
  display.printf("Steer: %3ddeg", manualServo);
  if (manualEditTarget == 1) display.print(" <");
  drawSteerBar(0, 42, 128, 8, manualServo);

  display.setCursor(0, 53);
  display.printf("Shift: %3ddeg", manualShifter);
  if (manualEditTarget == 2) display.print(" <");

  display.display();
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {

  Serial.begin(115200);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    if (hasSplash) {
      display.drawBitmap(0, 0, splashBitmap, 128, 64, SSD1306_WHITE);
    } else {
      drawCarIcon(58, 10);
      display.setTextSize(2);
      display.setCursor(20, 38);
      display.print("RC CAR");
      display.setTextSize(1);
    }
    display.display();
  }


  // Encoder pins
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);
  Serial.printf("Boot state — CLK:%d DT:%d\n", digitalRead(ENC_CLK), digitalRead(ENC_DT));

  // ESC arm
  motorESC.attach(MOTOR_PIN, 1000, 2000);
  motorESC.writeMicroseconds(1000);
  delay(2000);

  // Servo
  steeringServo.attach(SERVO_PIN, 1000, 2000);
  steeringServo.write(cfg.servoCenter);

  // Shifter servo
  shifterServo.attach(SHIFTER_PIN, 1000, 2000);
  shifterServo.write(90);


  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  Serial.println("=== Ready ===");
};

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  int delta = readEncoder();
  int btn = readButton();
  if (delta != 0 || btn != 0) lastActivityTime = millis();
  BP32.update();

  if (currentScreen != SCR_DRIVE && millis() - lastActivityTime > MENU_TIMEOUT_MS) {
    if (currentScreen == SCR_MANUAL_CONTROL) {
      motorESC.writeMicroseconds(1000);
      steeringServo.write(cfg.servoCenter);
      shifterServo.write(90);
    }
    currentScreen = SCR_DRIVE;
    menuIndex = 0;
  }

  // ── Controller input ─────────────────────────────────────────────────────
  if (ctrlConnected && dualshock && dualshock->isConnected()) {
    lxVal = dualshock->axisX();
    ryVal = dualshock->axisRY();
    r2Val = dualshock->throttle();
    l2Val = dualshock->brake();

    bool startPressed = dualshock->miscStart();
    if (startPressed && !lastStartState) {
      expMode = !expMode;
      smoothSignal = 1000;
      Serial.printf("Exp mode: %s\n", expMode ? "ON" : "OFF");
    }
    lastStartState = startPressed;

    bool dpadUp = dualshock->dpad() & DPAD_UP;
    bool dpadDown = dualshock->dpad() & DPAD_DOWN;
    if (dpadUp && !lastDpadUp) {
      if (currentGear == GEAR_R) currentGear = GEAR_N;
      else if (currentGear == GEAR_N) currentGear = GEAR_D;
    }
    if (dpadDown && !lastDpadDown) {
      if (currentGear == GEAR_D) currentGear = GEAR_N;
      else if (currentGear == GEAR_N) currentGear = GEAR_R;
    }
    lastDpadUp = dpadUp;
    lastDpadDown = dpadDown;

    shifterAngle = gearToAngle(currentGear);
    handbrake = l2Val > 50;

    bool l1Pressed = dualshock->buttons() & BUTTON_SHOULDER_L;
    bool r1Pressed = dualshock->buttons() & BUTTON_SHOULDER_R;
    if (l1Pressed && !lastL1 && currentPreset > 0) {
      currentPreset--;
      cfg = presets[currentPreset];
      presetAlertTime = millis();
    }
    if (r1Pressed && !lastR1 && currentPreset < presetCount - 1) {
      currentPreset++;
      cfg = presets[currentPreset];
      presetAlertTime = millis();
    }
    lastL1 = l1Pressed;
    lastR1 = r1Pressed;

    int rawAngle;
    if (lxVal >= 0) {
      // stick right: center → max
      rawAngle = cfg.servoReverse
        ? map(lxVal, 0, 511, cfg.servoCenter, cfg.servoMin)
        : map(lxVal, 0, 511, cfg.servoCenter, cfg.servoMax);
    } else {
      // stick left: center → min
      rawAngle = cfg.servoReverse
        ? map(lxVal, -512, 0, cfg.servoMax, cfg.servoCenter)
        : map(lxVal, -512, 0, cfg.servoMin, cfg.servoCenter);
    }
    servoAngle = constrain(rawAngle, cfg.servoMin, cfg.servoMax);

    if (currentScreen != SCR_CTRL_MONITOR && currentScreen != SCR_MANUAL_CONTROL) {
      if (expMode) {
        int target = constrain(map(r2Val, 0, 1020, 1000, cfg.maxTiming), 1000, cfg.maxTiming);
        float stickPct = r2Val / 1020.0f;
        smoothSignal += (stickPct > 0.02f)
                          ? (target - smoothSignal) * cfg.expAccelRate
                          : (1000 - smoothSignal) * cfg.expDecayRate;
        motorSignal = constrain((int)smoothSignal, 1000, cfg.maxTiming);
      } else {
        motorSignal = constrain(map(r2Val, 0, 1020, 1000, cfg.maxTiming), 1000, cfg.maxTiming);
        smoothSignal = motorSignal;
      }
      if (motorSignal < 1000 + cfg.deadzone) motorSignal = 1000;
      if (handbrake) motorSignal = 1000;
    }
  } else {
    lxVal = ryVal = r2Val = l2Val = 0;
    motorSignal = 1000;
    smoothSignal = 1000;
    servoAngle = cfg.servoCenter;
    handbrake = false;
  }

  if (currentScreen == SCR_MANUAL_CONTROL) {
    motorSignal  = manualMotor;
    servoAngle   = manualServo;
    shifterAngle = manualShifter;
  }

  motorESC.writeMicroseconds(motorSignal);
  steeringServo.write(servoAngle);
  shifterServo.write(shifterAngle);

  // ── Menu ─────────────────────────────────────────────────────────────────
  switch (currentScreen) {

    case SCR_DRIVE:
      if (btn == 1) {
        currentScreen = SCR_MAIN_MENU;
        menuIndex = 0;
      }
      updateDriveDisplay();
      break;

    case SCR_MAIN_MENU:
      menuIndex = constrain(menuIndex + delta, 0, mainMenuCount - 1);
      if (btn == 1) {
        switch (menuIndex) {
          case 0:
            currentScreen = SCR_MOTOR_MENU;
            menuIndex = 0;
            break;
          case 1:
            currentScreen = SCR_SERVO_MENU;
            menuIndex = 0;
            break;
          case 2:
            currentScreen = SCR_PRESET_MENU;
            menuIndex = currentPreset;
            break;
          case 3: currentScreen = SCR_CTRL_MONITOR; break;
          case 4:
            currentScreen    = SCR_MANUAL_CONTROL;
            manualMotor      = 1000;
            manualServo      = cfg.servoCenter;
            manualShifter    = 90;
            manualEditTarget = 0;
            break;
        }
      }
      if (btn == 2) {
        currentScreen = SCR_DRIVE;
        menuIndex = 0;
      }
      if (currentScreen == SCR_MAIN_MENU)
        drawScrollMenu("Main Menu", mainMenuItems, mainMenuCount, menuIndex);
      break;

    case SCR_MOTOR_MENU:
      menuIndex = constrain(menuIndex + delta, 0, motorMenuCount - 1);
      if (btn == 1) {
        editingItem = menuIndex;
        switch (menuIndex) {
          case 0: enterEditInt("Max timing", cfg.maxTiming, 1000, 2000, 10); break;
          case 1: enterEditInt("Deadzone", cfg.deadzone, 0, 200, 1); break;
          case 2: enterEditFloat("Accel rate", cfg.expAccelRate, 0.01, 1.0, 0.01); break;
          case 3: enterEditFloat("Decay rate", cfg.expDecayRate, 0.01, 1.0, 0.01); break;
        }
      }
      if (btn == 2) {
        currentScreen = SCR_MAIN_MENU;
        menuIndex = 0;
      }
      if (currentScreen == SCR_MOTOR_MENU)
        drawScrollMenu("Motor", motorMenuItems, motorMenuCount, menuIndex);
      break;

    case SCR_SERVO_MENU:
      menuIndex = constrain(menuIndex + delta, 0, servoMenuCount - 1);
      if (btn == 1) {
        editingItem = menuIndex;
        switch (menuIndex) {
          case 0: enterEditInt("Servo min", cfg.servoMin, 0, 90, 1); break;
          case 1: enterEditInt("Servo max", cfg.servoMax, 90, 180, 1); break;
          case 2: enterEditBool("Reverse", cfg.servoReverse); break;
          case 3: enterEditInt("Servo center", cfg.servoCenter, 80, 100, 1); break;
        }
      }
      if (btn == 2) {
        currentScreen = SCR_MAIN_MENU;
        menuIndex = 0;
      }
      if (currentScreen == SCR_SERVO_MENU)
        drawScrollMenu("Servo", servoMenuItems, servoMenuCount, menuIndex);
      break;

    case SCR_PRESET_MENU:
      menuIndex = constrain(menuIndex + delta, 0, presetCount - 1);
      if (btn == 1) {
        currentPreset = menuIndex;
        cfg = presets[currentPreset];
        presetAlertTime = millis();
        currentScreen = SCR_DRIVE;
        menuIndex = 0;
      }
      if (btn == 2) {
        currentScreen = SCR_MAIN_MENU;
        menuIndex = 2;
      }
      if (currentScreen == SCR_PRESET_MENU)
        drawScrollMenu("Mode", presetNames, presetCount, menuIndex);
      break;

    case SCR_CTRL_MONITOR:
      if (btn == 2) {
        currentScreen = SCR_MAIN_MENU;
        menuIndex = 3;
      }
      updateCtrlMonitor();
      break;
    
    case SCR_MANUAL_CONTROL:
      if (delta != 0) {
        switch (manualEditTarget) {
          case 0: manualMotor   = constrain(manualMotor + delta * 20, 1000, cfg.maxTiming); break;
          case 1: manualServo   = constrain(manualServo + delta * 10, cfg.servoMin, cfg.servoMax); break;
          case 2: manualShifter = constrain(manualShifter + delta * 90, 0, 180); break;
        }
      }
      if (btn == 1) manualEditTarget = (manualEditTarget + 1) % 3;
      if (btn == 2) {
        motorESC.writeMicroseconds(1000);
        steeringServo.write(cfg.servoCenter);
        shifterServo.write(90);
        currentScreen = SCR_MAIN_MENU;
        menuIndex = 4;
      }
      updateManualControl();
      break;

    case SCR_EDIT_VALUE:
      if (delta != 0) {
        if (edit.type == EDIT_INT)
          edit.intVal = constrain(edit.intVal + delta * edit.intStep, edit.intMin, edit.intMax);
        else if (edit.type == EDIT_FLOAT) {
          edit.floatVal = constrain(edit.floatVal + delta * edit.floatStep, edit.floatMin, edit.floatMax);
          edit.floatVal = roundf(edit.floatVal * 100) / 100;
        } else if (edit.type == EDIT_BOOL)
          edit.boolVal = !edit.boolVal;
      }
      if (btn == 1) {
        applyEdit();
        currentScreen = prevMenu;
        menuIndex = editingItem;
      }
      if (btn == 2) {
        currentScreen = prevMenu;
        menuIndex = editingItem;
      }
      drawEditScreen();
      break;
  }

  delay(10);
}

