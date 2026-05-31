#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include "splash.h"

// ─── Pins ────────────────────────────────────────────────────────────────────
#define MOTOR_PIN   19
#define SERVO_PIN   18
#define SHIFTER_PIN 23
#define OLED_SDA    21
#define OLED_SCL    22
#define ENC_CLK     15
#define ENC_DT       4
#define ENC_BTN      5

// ─── Animation ───────────────────────────────────────────────────────────────
#define FRAME_DELAY  42
#define FRAME_WIDTH  64
#define FRAME_HEIGHT 64
#define FRAME_COUNT  (sizeof(mainMenuAnimation) / sizeof(mainMenuAnimation[0]))

// ─── Timeouts ────────────────────────────────────────────────────────────────
#define MENU_TIMEOUT_MS 10000

// ─── Button edge detector ────────────────────────────────────────────────────
struct Button {
  bool _last = false;
  bool pressed(bool current)  { bool e = current && !_last; _last = current; return e; }
  bool released(bool current) { bool e = !current && _last; _last = current; return e; }
  void update(bool current)   { _last = current; }
};

// ─── Config ──────────────────────────────────────────────────────────────────
struct Config {
  const char* name;
  int   maxTiming;
  int   deadzone;
  int   steerDeadzone;
  float expAccelRate;
  float expDecayRate;
  int   servoCenter;
  int   servoMin;
  int   servoMax;
  bool  servoReverse;
  uint8_t colorR, colorG, colorB;
};

// ─── Presets ─────────────────────────────────────────────────────────────────
// 1800 - lego gears start grinding and esp32 power can bounce, do not set > 1800!
const Config presets[] = {
  //  NAME      MAXTIMING   DEADZONE    STEERDZ   ACCRATE   DECRATE   SERVOCENTER   SERVOMIN    SERVOMAX    SERVOREVERSE    COLOR:R   COLOR:G   COLOR:B
  {   "Drive",  1200,       15,         30,       0.10f,    0.05f,    90,           0,          180,        false,          255,      180,      0       },
  {   "Drift",  1430,       5,          30,       0.21f,    0.05f,    90,           0,          180,        false,          199,      77,       155     },
  {   "Race",   1500,       5,          30,       0.50f,    0.15f,    90,           30,         150,        false,          255,      0,        0       },
};
const int presetCount = sizeof(presets) / sizeof(presets[0]);
int    currentPreset  = 1;
// load Drift as default
Config cfg            = presets[1];

// ─── OLED ────────────────────────────────────────────────────────────────────
Adafruit_SSD1306 display(128, 64, &Wire, -1);
const bool hasSplash  = true;
int        animFrame  = 0;
unsigned long lastFrameTime = 0;

// ─── Servo / ESC ─────────────────────────────────────────────────────────────
Servo motorESC;
Servo steeringServo;
Servo shifterServo;

// ─── Bluepad32 ───────────────────────────────────────────────────────────────
ControllerPtr dualshock;
bool ctrlConnected = false;
int  lxVal = 0, ryVal = 0, r2Val = 0, l2Val = 0;

// ─── Vroom rumble state machine ───────────────────────────────────────────────
// Could be better :)
enum RumbleState { RUMBLE_IDLE, RUMBLE_VROOM1, RUMBLE_GAP, RUMBLE_VROOM2 };
RumbleState   rumbleState = RUMBLE_IDLE;
unsigned long rumbleTimer = 0;
#define VROOM_ON_MS  150
#define VROOM_GAP_MS  80

void startVroom() {
  rumbleState = RUMBLE_VROOM1;
  rumbleTimer = millis();
  if (dualshock) dualshock->playDualRumble(0, VROOM_ON_MS, 255, 255);
}

void updateVroom() {
  if (rumbleState == RUMBLE_IDLE) return;
  unsigned long now = millis();
  switch (rumbleState) {
    case RUMBLE_VROOM1:
      if (now - rumbleTimer >= VROOM_ON_MS) {
        rumbleState = RUMBLE_GAP;
        rumbleTimer = now;
        if (dualshock) dualshock->playDualRumble(0, VROOM_GAP_MS, 0, 0);
      }
      break;
    case RUMBLE_GAP:
      if (now - rumbleTimer >= VROOM_GAP_MS) {
        rumbleState = RUMBLE_VROOM2;
        rumbleTimer = now;
        if (dualshock) dualshock->playDualRumble(0, VROOM_ON_MS, 255, 255);
      }
      break;
    case RUMBLE_VROOM2:
      if (now - rumbleTimer >= VROOM_ON_MS) rumbleState = RUMBLE_IDLE;
      break;
    default: break;
  }
}

void applyLED() {
  if (!dualshock) return;
  dualshock->setColorLED(cfg.colorR, cfg.colorG, cfg.colorB);
}

void onConnectedController(ControllerPtr ctl) {
  Serial.println(">>> Controller connected!");
  dualshock     = ctl;
  ctrlConnected = true;
  applyLED();
  startVroom();
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println(">>> Controller disconnected!");
  if (ctl == dualshock) dualshock = nullptr;
  ctrlConnected = false;
  lxVal = ryVal = r2Val = l2Val = 0;
  rumbleState = RUMBLE_IDLE;
}

// ─── Shifter ─────────────────────────────────────────────────────────────────
enum Gear { GEAR_D, GEAR_N, GEAR_R };
Gear currentGear = GEAR_N;

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

// ─── Global state ─────────────────────────────────────────────────────────────
int   motorSignal  = 1000;
int   servoAngle   = cfg.servoCenter;
int   shifterAngle = 90;
bool  expMode      = false;
bool  handbrake    = false;
float smoothSignal = 1000;

int   manualMotor      = 1000;
int   manualServo      = 90;
int   manualShifter    = 90;
int   manualEditTarget = 0;

unsigned long lastActivityTime = 0;

// ─── Controller buttons ───────────────────────────────────────────────────────
Button btnL1, btnR1, btnDpadUp, btnDpadDown, btnStart, btnHandbrake;

// ─── Encoder ──────────────────────────────────────────────────────────────────
uint8_t stableRead(int pin) {
  uint8_t count = 0;
  for (int i = 0; i < 3; i++) {
    count += digitalRead(pin);
    delayMicroseconds(50);
  }
  return (count >= 2) ? 1 : 0;
}

int readEncoder() {
  static uint8_t prevState  = 3;
  static int     accum      = 0;
  static unsigned long lastTick   = 0;
  static unsigned long lastChange = 0;

  uint8_t clk   = stableRead(ENC_CLK);
  uint8_t dt    = stableRead(ENC_DT);
  uint8_t state = (clk << 1) | dt;

  if (state != prevState && millis() - lastChange >= 2) {
    if (prevState == 3 && state == 1) accum++;
    if (prevState == 1 && state == 0) accum++;
    if (prevState == 3 && state == 2) accum--;
    if (prevState == 2 && state == 0) accum--;
    prevState  = state;
    lastChange = millis();
    lastTick   = millis();
  }

  int delta = 0;
  unsigned long age = millis() - lastTick;
  int threshold = (age < 80) ? 1 : 2;
  if (accum >=  threshold) { delta =  1; accum = 0; }
  if (accum <= -threshold) { delta = -1; accum = 0; }
  return delta;
}

// ─── Encoder button — debounce + long press ───────────────────────────────────
#define LONG_PRESS_MS  800
unsigned long btnPressTime     = 0;
bool          btnWasPressed    = false;
bool          longPressHandled = false;

// returns 1 = short click, 2 = long press, 0 = nothing
int readButton() {
  bool pressed = (digitalRead(ENC_BTN) == LOW);
  int  result  = 0;
  if (pressed && !btnWasPressed) {
    btnPressTime     = millis();
    btnWasPressed    = true;
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

// ─── Screens ──────────────────────────────────────────────────────────────────
enum Screen {
  SCR_DRIVE,
  SCR_MAIN_MENU,
  SCR_MOTOR_MENU,
  SCR_SERVO_MENU,
  SCR_CTRL_MONITOR,
  SCR_MANUAL_CONTROL,
  SCR_EDIT_VALUE
};
Screen currentScreen = SCR_DRIVE;

// ─── Edit state ───────────────────────────────────────────────────────────────
enum EditType { EDIT_INT, EDIT_FLOAT, EDIT_BOOL };
struct EditState {
  const char* label;
  EditType    type;
  int   intVal,   intMin,   intMax,   intStep;
  float floatVal, floatMin, floatMax, floatStep;
  bool  boolVal;
} edit;

Screen prevMenu    = SCR_MAIN_MENU;
int    editingItem = 0;
int    menuIndex   = 0;

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
      case 0: cfg.maxTiming    = edit.intVal;   break;
      case 1: cfg.deadzone     = edit.intVal;   break;
      case 2: cfg.expAccelRate = edit.floatVal; break;
      case 3: cfg.expDecayRate = edit.floatVal; break;
    }
  } else if (prevMenu == SCR_SERVO_MENU) {
    switch (editingItem) {
      case 0: cfg.servoMin      = edit.intVal;  break;
      case 1: cfg.servoMax      = edit.intVal;  break;
      case 2: cfg.servoReverse  = edit.boolVal; break;
      case 3: cfg.servoCenter   = edit.intVal;  break;
      case 4: cfg.steerDeadzone = edit.intVal;  break;
    }
  }
}

// ─── Menu item lists ──────────────────────────────────────────────────────────
const char* mainMenuItems[]  = { "Motor", "Servo", "Controller", "Manual" };
const int   mainMenuCount    = 4;
const char* motorMenuItems[] = { "Max timing", "Deadzone", "Accel rate", "Decay rate" };
const int   motorMenuCount   = 4;
const char* servoMenuItems[] = { "Servo min", "Servo max", "Reverse", "Center", "Steer DZ" };
const int   servoMenuCount   = 5;

// ─── Display helpers ──────────────────────────────────────────────────────────
void drawMotorBar(int x, int y, int w, int h, int signal) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  float pct  = (signal - 1000) / (float)(cfg.maxTiming - 1000);
  int   fill = constrain((int)(pct * (w - 4)), 0, w - 4);
  if (fill > 0) display.fillRect(x + 2, y + 2, fill, h - 4, SSD1306_WHITE);
}

void drawSteerBar(int x, int y, int w, int h, int angle) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int blobW = 5;
  int pos   = cfg.servoReverse
    ? map(angle, cfg.servoMax, cfg.servoMin, x + 2, x + w - 2 - blobW)
    : map(angle, cfg.servoMin, cfg.servoMax, x + 2, x + w - 2 - blobW);
  pos = constrain(pos, x + 2, x + w - 2 - blobW);
  display.fillRect(pos, y + 1, blobW, h - 2, SSD1306_WHITE);
}

void drawScrollMenu(const char* title, const char** items, int count, int selected) {
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(4, 2);
  display.print(title);
  display.setTextColor(SSD1306_WHITE);
  int start = max(0, min(selected, count - 4));
  for (int i = start; i < min(start + 4, count); i++) {
    int y = 13 + (i - start) * 13;
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
  // edit values screen
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(4, 2);
  display.print(edit.label);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(20, 24);
  if      (edit.type == EDIT_INT)   display.printf("%d",   edit.intVal);
  else if (edit.type == EDIT_FLOAT) display.printf("%.2f", edit.floatVal);
  else                              display.print(edit.boolVal ? "YES" : "NO");
  display.setTextSize(1);
  display.fillRect(0, 55, 128, 9, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(4, 56);
  display.print("[Save]");
  display.setCursor(87, 56);
  display.print("[Back]");
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void updateDriveDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (ctrlConnected) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(presets[currentPreset].name);

    display.setCursor(108, 0);
    display.print(gearChar(currentGear));

    display.setTextSize(1);
    display.setCursor(0, 18);
    if (expMode)   display.print("EXP ");
    if (handbrake) display.print("BRK");

    display.setCursor(0, 28);
    display.print("Motor");
    drawMotorBar(0, 37, 128, 8, motorSignal);

    display.setCursor(0, 48);
    display.print("Steer");
    drawSteerBar(0, 57, 128, 7, servoAngle);

  } else {
    unsigned long now = millis();
    if (now - lastFrameTime >= FRAME_DELAY) {
      animFrame = (animFrame + 1) % FRAME_COUNT;
      lastFrameTime = now;
    }
    display.drawBitmap(32, 0, mainMenuAnimation[animFrame], FRAME_WIDTH, FRAME_HEIGHT, SSD1306_WHITE);
  }

  display.display();
}

void updateCtrlMonitor() {
  display.clearDisplay();
  display.setTextSize(1);
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(4, 2);
  display.print("Controller");
  display.setTextColor(SSD1306_WHITE);
  if (ctrlConnected) {
    display.setCursor(0, 14);
    display.printf("LX:%5d  RY:%5d", lxVal, ryVal);
    display.setCursor(0, 25);
    display.printf("R2:%5d  L2:%5d", r2Val, l2Val);
    display.setCursor(0, 36);
    display.printf("EXP:%-3s  Gear:[%c]", expMode ? "ON" : "OFF", gearChar(currentGear));
    display.setCursor(0, 47);
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
  display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(4, 2);
  display.printf("Manual [%s]", targetNames[manualEditTarget]);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 14);
  display.printf("Motor: %4d", manualMotor);
  if (manualEditTarget == 0) display.print(" <");
  drawMotorBar(0, 23, 128, 8, manualMotor);

  display.setCursor(0, 34);
  display.printf("Steer: %3ddeg", manualServo);
  if (manualEditTarget == 1) display.print(" <");
  drawSteerBar(0, 43, 128, 8, manualServo);

  display.setCursor(0, 54);
  display.printf("Shift: %3ddeg", manualShifter);
  if (manualEditTarget == 2) display.print(" <");

  display.display();
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Connect and init screen, draw splash screen
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    if (hasSplash) {
      display.drawBitmap(0, 0, splashBitmap, 128, 64, SSD1306_WHITE);
    } else {
      // Fallback if image not present
      display.setTextSize(2);
      display.setCursor(20, 38);
      display.print("RC CAR");
      display.setTextSize(1);
    }
    display.display();
  }

  // Init encoder
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_BTN, INPUT_PULLUP);

  // Arm motor and servos
  motorESC.attach(MOTOR_PIN, 1000, 2000);
  motorESC.writeMicroseconds(1000);

  steeringServo.attach(SERVO_PIN, 1000, 2000);
  steeringServo.write(cfg.servoCenter);

  shifterServo.attach(SHIFTER_PIN, 1000, 2000);
  shifterServo.write(90);

  // Init BT and clear old keys (This esp can not save anything for some reason anyway)
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();

  Serial.println("=== Ready ===");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  int delta = readEncoder();
  int btn   = readButton();
  if (delta != 0 || btn != 0) lastActivityTime = millis();

  BP32.update();
  updateVroom();

  // Menu timeout — return to drive
  if (currentScreen != SCR_DRIVE && millis() - lastActivityTime > MENU_TIMEOUT_MS) {
    if (currentScreen == SCR_MANUAL_CONTROL) {
      motorESC.writeMicroseconds(1000);
      steeringServo.write(cfg.servoCenter);
      shifterServo.write(90);
    }
    currentScreen = SCR_DRIVE;
    menuIndex = 0;
  }

  // ── Controller input ──────────────────────────────────────────────────────
  if (ctrlConnected && dualshock && dualshock->isConnected()) {
    lxVal = dualshock->axisX();
    ryVal = dualshock->axisRY();
    r2Val = dualshock->throttle();
    l2Val = dualshock->brake();

    if (btnStart.pressed(dualshock->miscStart())) {
      expMode = !expMode;
      smoothSignal = 1000;
    }

    if (btnDpadUp.pressed(dualshock->dpad() & DPAD_UP)) {
      if      (currentGear == GEAR_R) currentGear = GEAR_N;
      else if (currentGear == GEAR_N) currentGear = GEAR_D;
    }
    if (btnDpadDown.pressed(dualshock->dpad() & DPAD_DOWN)) {
      if      (currentGear == GEAR_D) currentGear = GEAR_N;
      else if (currentGear == GEAR_N) currentGear = GEAR_R;
    }

    shifterAngle = gearToAngle(currentGear);
    handbrake    = l2Val > 50;

    if (btnL1.pressed(dualshock->buttons() & BUTTON_SHOULDER_L)) {
      if (currentPreset > 0) { currentPreset--; cfg = presets[currentPreset]; applyLED(); }
    }
    if (btnR1.pressed(dualshock->buttons() & BUTTON_SHOULDER_R)) {
      if (currentPreset < presetCount - 1) { currentPreset++; cfg = presets[currentPreset]; applyLED(); }
    }

    if (btnHandbrake.pressed(handbrake))  { if (dualshock) dualshock->playDualRumble(0, 65535, 80, 0); }
    if (btnHandbrake.released(handbrake)) { if (dualshock) dualshock->playDualRumble(0, 0, 0, 0); }
    btnHandbrake.update(handbrake);

    int rawAngle;
    if (abs(lxVal) <= cfg.steerDeadzone) {
      rawAngle = cfg.servoCenter;
    } else if (lxVal > cfg.steerDeadzone) {
      rawAngle = cfg.servoReverse
        ? map(lxVal, cfg.steerDeadzone, 511, cfg.servoCenter, cfg.servoMin)
        : map(lxVal, cfg.steerDeadzone, 511, cfg.servoCenter, cfg.servoMax);
    } else {
      rawAngle = cfg.servoReverse
        ? map(lxVal, -512, -cfg.steerDeadzone, cfg.servoMax, cfg.servoCenter)
        : map(lxVal, -512, -cfg.steerDeadzone, cfg.servoMin, cfg.servoCenter);
    }
    servoAngle = constrain(rawAngle, cfg.servoMin, cfg.servoMax);

    if (currentScreen != SCR_CTRL_MONITOR && currentScreen != SCR_MANUAL_CONTROL) {
      if (expMode) {
        int   target   = constrain(map(r2Val, 0, 1020, 1000, cfg.maxTiming), 1000, cfg.maxTiming);
        float stickPct = r2Val / 1020.0f;
        smoothSignal  += (stickPct > 0.02f)
                           ? (target - smoothSignal) * cfg.expAccelRate
                           : (1000   - smoothSignal) * cfg.expDecayRate;
        motorSignal    = constrain((int)smoothSignal, 1000, cfg.maxTiming);
      } else {
        motorSignal  = constrain(map(r2Val, 0, 1020, 1000, cfg.maxTiming), 1000, cfg.maxTiming);
        smoothSignal = motorSignal;
      }
      if (motorSignal < 1000 + cfg.deadzone) motorSignal = 1000;
      if (handbrake) motorSignal = 1000;
    }

  } else {
    btnStart.update(false);
    btnDpadUp.update(false);
    btnDpadDown.update(false);
    btnL1.update(false);
    btnR1.update(false);
    btnHandbrake.update(false);

    lxVal = ryVal = r2Val = l2Val = 0;
    motorSignal  = 1000;
    smoothSignal = 1000;
    servoAngle   = cfg.servoCenter;
    handbrake    = false;
  }

  if (currentScreen == SCR_MANUAL_CONTROL) {
    motorSignal  = manualMotor;
    servoAngle   = manualServo;
    shifterAngle = manualShifter;
  }

  motorESC.writeMicroseconds(motorSignal);
  steeringServo.write(servoAngle);
  shifterServo.write(shifterAngle);

  // ── Menu ──────────────────────────────────────────────────────────────────
  switch (currentScreen) {

    case SCR_DRIVE:
      if (btn == 1) { currentScreen = SCR_MAIN_MENU; menuIndex = 0; }
      updateDriveDisplay();
      break;

    case SCR_MAIN_MENU:
      menuIndex = constrain(menuIndex + delta, 0, mainMenuCount - 1);
      if (btn == 1) {
        switch (menuIndex) {
          case 0: currentScreen = SCR_MOTOR_MENU;  menuIndex = 0; break;
          case 1: currentScreen = SCR_SERVO_MENU;  menuIndex = 0; break;
          case 2: currentScreen = SCR_CTRL_MONITOR; break;
          case 3:
            currentScreen    = SCR_MANUAL_CONTROL;
            manualMotor      = 1000;
            manualServo      = cfg.servoCenter;
            manualShifter    = 90;
            manualEditTarget = 0;
            break;
        }
      }
      if (btn == 2) { currentScreen = SCR_DRIVE; menuIndex = 0; }
      if (currentScreen == SCR_MAIN_MENU)
        drawScrollMenu("Settings", mainMenuItems, mainMenuCount, menuIndex);
      break;

    case SCR_MOTOR_MENU:
      menuIndex = constrain(menuIndex + delta, 0, motorMenuCount - 1);
      if (btn == 1) {
        editingItem = menuIndex;
        switch (menuIndex) {
          case 0: enterEditInt("Max timing",   cfg.maxTiming,    1000, 2000, 10);   break;
          case 1: enterEditInt("Deadzone",     cfg.deadzone,        0,  200,  1);   break;
          case 2: enterEditFloat("Accel rate", cfg.expAccelRate, 0.01,  1.0, 0.01); break;
          case 3: enterEditFloat("Decay rate", cfg.expDecayRate, 0.01,  1.0, 0.01); break;
        }
      }
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 0; }
      if (currentScreen == SCR_MOTOR_MENU)
        drawScrollMenu("Motor", motorMenuItems, motorMenuCount, menuIndex);
      break;

    case SCR_SERVO_MENU:
      menuIndex = constrain(menuIndex + delta, 0, servoMenuCount - 1);
      if (btn == 1) {
        editingItem = menuIndex;
        switch (menuIndex) {
          case 0: enterEditInt("Servo min",    cfg.servoMin,        0,  90,  1); break;
          case 1: enterEditInt("Servo max",    cfg.servoMax,       90, 180,  1); break;
          case 2: enterEditBool("Reverse",     cfg.servoReverse);                break;
          case 3: enterEditInt("Center",       cfg.servoCenter,    80, 100,  1); break;
          case 4: enterEditInt("Steer DZ",     cfg.steerDeadzone,   0, 100,  1); break;
        }
      }
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 0; }
      if (currentScreen == SCR_SERVO_MENU)
        drawScrollMenu("Servo", servoMenuItems, servoMenuCount, menuIndex);
      break;

    case SCR_CTRL_MONITOR:
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 3; }
      updateCtrlMonitor();
      break;

    case SCR_MANUAL_CONTROL:
      if (delta != 0) {
        switch (manualEditTarget) {
          case 0: manualMotor   = constrain(manualMotor   + delta * 20, 1000, cfg.maxTiming);        break;
          case 1: manualServo   = constrain(manualServo   + delta * 10, cfg.servoMin, cfg.servoMax); break;
          case 2: manualShifter = constrain(manualShifter + delta * 90, 0, 180);                     break;
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
        if (edit.type == EDIT_INT) {
          edit.intVal = constrain(edit.intVal + delta * edit.intStep, edit.intMin, edit.intMax);
        } else if (edit.type == EDIT_FLOAT) {
          edit.floatVal = constrain(edit.floatVal + delta * edit.floatStep, edit.floatMin, edit.floatMax);
          edit.floatVal = roundf(edit.floatVal * 100) / 100;
        } else {
          edit.boolVal = !edit.boolVal;
        }
      }
      if (btn == 1) { applyEdit(); currentScreen = prevMenu; menuIndex = editingItem; }
      if (btn == 2) {              currentScreen = prevMenu; menuIndex = editingItem; }
      drawEditScreen();
      break;
  }

  delay(10);
}
