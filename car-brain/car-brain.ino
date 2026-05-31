#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include <WiFi.h>
#include <WebSocketsServer_Generic.h>
#include <ArduinoJson.h>
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



// ─── WebSocket ───────────────────────────────────────────────────────────────
#define WS_PORT            81
#define TELEM_INTERVAL_MS  150
#define WS_CONTROL_TIMEOUT 50000

WebSocketsServer webSocket(WS_PORT);
unsigned long lastTelemTime  = 0;
bool  wsClientConnected      = false;
float wsThrottle             = 0;
float wsSteer                = 0;
bool  wsActive               = false;
unsigned long lastWsControl  = 0;

// ─── Main menu animation ────────────────────────────────────────────────────
#define FRAME_DELAY 42
#define FRAME_WIDTH 64
#define FRAME_HEIGHT 64
#define FRAME_COUNT (sizeof(mainMenuAnimation) / sizeof(mainMenuAnimation[0]))

// ─── Timeout to exit from settings ──────────────────────────────────────────
#define MENU_TIMEOUT_MS 10000

// ─── Config ─────────────────────────────────────────────────────────────────
struct Config {
  int maxTiming;
  int deadzone;
  int steerDeadzone;
  float expAccelRate;
  float expDecayRate;
  int servoCenter;
  int servoMin;
  int servoMax;
  bool servoReverse;
};

// ─── Presets ─────────────────────────────────────────────────────────────────
const char* presetNames[] = { "Drive", "Drift", "Race" };
// 1800 - lego gears start grinding and esp32 power can bounce, do not set > 1800!
const Config presets[] = {
  { 1200, 15, 30, 0.10f, 0.05f, 90, 0, 180, false },
  { 1440, 5, 30, 0.22f, 0.05f, 90, 0, 180, false },
  { 1500,  5, 30, 0.50f, 0.15f, 90, 30, 150, false },
};
const int presetCount = 3;
int currentPreset = 1;
Config cfg = presets[1];

// WS config — loaded from console on WS connect
Config wsConfig = { 1400, 5, 30, 0.20f, 0.08f, 90, 0, 180, false };
Config preWsConfig;   // stash BT config to restore on WS disconnect
int    preWsPreset;

// LED colors per preset: Drive=green, Drift=yellow, Race=red
const uint8_t presetColors[][3] = {
  {   0, 255,   0 },
  { 255, 180,   0 },
  { 255,   0,   0 },
};

// ─── OLED related stuff ──────────────────────────────────────────────────────
Adafruit_SSD1306 display(128, 64, &Wire, -1);
const bool hasSplash = true;
int animFrame = 0;
unsigned long lastFrameTime = 0;

// ─── Servo / ESC ─────────────────────────────────────────────────────────────
Servo motorESC;
Servo steeringServo;
Servo shifterServo;

// ─── Bluepad32 ───────────────────────────────────────────────────────────────
ControllerPtr dualshock;
bool ctrlConnected = false;
int lxVal = 0, ryVal = 0, r2Val = 0, l2Val = 0;

// ─── Vroom rumble state machine ──────────────────────────────────────────────
enum RumbleState { RUMBLE_IDLE, RUMBLE_VROOM1, RUMBLE_GAP, RUMBLE_VROOM2 };
RumbleState  rumbleState = RUMBLE_IDLE;
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
      if (now - rumbleTimer >= VROOM_ON_MS) {
        rumbleState = RUMBLE_IDLE;
      }
      break;
    default: break;
  }
}

void applyLED() {
  if (!dualshock) return;
  dualshock->setColorLED(presetColors[currentPreset][0],
                         presetColors[currentPreset][1],
                         presetColors[currentPreset][2]);
}

void onConnectedController(ControllerPtr ctl) {
  Serial.println(">>> Controller connected!");
  dualshock = ctl;
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

void disconnectController() {
  if (dualshock && ctrlConnected) {
    Serial.println("[WS] Disconnecting BT controller");
    dualshock->disconnect();
    dualshock = nullptr;
    ctrlConnected = false;
    lxVal = ryVal = r2Val = l2Val = 0;
    rumbleState = RUMBLE_IDLE;
  }
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

// ─── Global state ────────────────────────────────────────────────────────────
int motorSignal = 1000;
int servoAngle = cfg.servoCenter;
int shifterAngle = 90;
bool expMode = false;
bool handbrake = false;

float smoothSignal = 1000;
int   manualMotor     = 1000;
int   manualServo     = 90;
int   manualShifter   = 90;
int   manualEditTarget = 0;
unsigned long lastActivityTime = 0;

bool lastL1 = false, lastR1 = false;
bool lastDpadUp = false, lastDpadDown = false;
bool lastStartState = false;
bool lastHandbrake = false;

// ─── WebSocket handlers ──────────────────────────────────────────────────────
void broadcastTelemetry() {
  StaticJsonDocument<512> doc;
  doc["type"]          = "telemetry";
  doc["motor"]         = motorSignal;
  doc["servo"]         = servoAngle;
  doc["gear"]          = String(gearChar(currentGear));
  doc["preset"]        = wsClientConnected ? "WS" : presetNames[currentPreset];
  doc["presetIndex"]   = currentPreset;
  doc["exp"]           = expMode;
  doc["handbrake"]     = handbrake;
  doc["maxTiming"]     = cfg.maxTiming;
  doc["deadzone"]      = cfg.deadzone;
  doc["steerDeadzone"] = cfg.steerDeadzone;
  doc["servoCenter"]   = cfg.servoCenter;
  doc["servoMin"]      = cfg.servoMin;
  doc["servoMax"]      = cfg.servoMax;
  doc["expAccelRate"]  = cfg.expAccelRate;
  doc["expDecayRate"]  = cfg.expDecayRate;
  doc["controller"]    = ctrlConnected;

  String out;
  serializeJson(doc, out);
  webSocket.broadcastTXT(out);
}

void applyWsConfig(StaticJsonDocument<512>& doc) {
  wsConfig.maxTiming     = doc["maxTiming"]     | wsConfig.maxTiming;
  wsConfig.deadzone      = doc["deadzone"]      | wsConfig.deadzone;
  wsConfig.steerDeadzone = doc["steerDeadzone"]  | wsConfig.steerDeadzone;
  wsConfig.expAccelRate  = doc["expAccelRate"]   | wsConfig.expAccelRate;
  wsConfig.expDecayRate  = doc["expDecayRate"]   | wsConfig.expDecayRate;
  wsConfig.servoCenter   = doc["servoCenter"]    | wsConfig.servoCenter;
  wsConfig.servoMin      = doc["servoMin"]       | wsConfig.servoMin;
  wsConfig.servoMax      = doc["servoMax"]       | wsConfig.servoMax;
  wsConfig.servoReverse  = doc["servoReverse"]   | wsConfig.servoReverse;
  cfg = wsConfig;
  Serial.println("[WS] Full config applied");
}

void handleWsCommand(uint8_t num, const char* payload) {
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, payload)) return;

  const char* type = doc["type"];
  if (!type) return;

  if (strcmp(type, "control") == 0) {
    wsThrottle = doc["throttle"] | 0.0f;
    wsSteer    = doc["steer"]    | 0.0f;
    handbrake  = doc["handbrake"] | false;
    lastWsControl = millis();
    wsActive = true;
  }
  else if (strcmp(type, "gear") == 0) {
    const char* val = doc["value"];
    if (val) {
      switch (val[0]) {
        case 'D': currentGear = GEAR_D; break;
        case 'R': currentGear = GEAR_R; break;
        default:  currentGear = GEAR_N; break;
      }
    }
  }
  else if (strcmp(type, "preset") == 0) {
    const char* dir = doc["direction"];
    if (dir) {
      if (strcmp(dir, "next") == 0 && currentPreset < presetCount - 1) currentPreset++;
      else if (strcmp(dir, "prev") == 0 && currentPreset > 0) currentPreset--;
      cfg = presets[currentPreset];
    }
  }
  else if (strcmp(type, "toggle") == 0) {
    const char* feature = doc["feature"];
    if (feature && strcmp(feature, "exp") == 0) {
      expMode = !expMode;
      smoothSignal = 1000;
    }
  }
  else if (strcmp(type, "config") == 0) {
    const char* key = doc["key"];
    if (!key) return;
    if (strcmp(key, "maxTiming") == 0)       cfg.maxTiming     = doc["value"] | cfg.maxTiming;
    else if (strcmp(key, "deadzone") == 0)    cfg.deadzone      = doc["value"] | cfg.deadzone;
    else if (strcmp(key, "steerDeadzone") == 0) cfg.steerDeadzone = doc["value"] | cfg.steerDeadzone;
    else if (strcmp(key, "servoCenter") == 0) cfg.servoCenter   = doc["value"] | cfg.servoCenter;
    else if (strcmp(key, "servoMin") == 0)    cfg.servoMin      = doc["value"] | cfg.servoMin;
    else if (strcmp(key, "servoMax") == 0)    cfg.servoMax      = doc["value"] | cfg.servoMax;
    else if (strcmp(key, "expAccelRate") == 0) cfg.expAccelRate  = doc["value"] | cfg.expAccelRate;
    else if (strcmp(key, "expDecayRate") == 0) cfg.expDecayRate  = doc["value"] | cfg.expDecayRate;
    wsConfig = cfg;
    Serial.printf("[WS] Config %s updated\n", key);
  }
  else if (strcmp(type, "ws_config") == 0) {
    applyWsConfig(doc);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client %u connected\n", num);
      wsClientConnected = true;
      // Stash current BT config, switch to WS config
      preWsConfig = cfg;
      preWsPreset = currentPreset;
      cfg = wsConfig;
      // Disconnect BT controller
      disconnectController();
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client %u disconnected\n", num);
      wsClientConnected = false;
      wsActive = false;
      wsThrottle = 0;
      wsSteer = 0;
      motorSignal = 1000;
      smoothSignal = 1000;
      // Restore BT config
      cfg = preWsConfig;
      currentPreset = preWsPreset;
      Serial.println("[WS] Restored BT config");
      break;
    case WStype_TEXT:
      handleWsCommand(num, (const char*)payload);
      break;
    default: break;
  }
}

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
  int threshold = (age < 80) ? 1 : 2;

  if (accum >=  threshold) { delta =  1; accum = 0; }
  if (accum <= -threshold) { delta = -1; accum = 0; }
  return delta;
}

// ─── Button — debounce + long press ──────────────────────────────────────────
#define LONG_PRESS_MS 800
unsigned long btnPressTime = 0;
bool btnWasPressed = false;
bool longPressHandled = false;

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
const char* servoMenuItems[] = { "Servo min", "Servo max", "Reverse", "Center", "Deadzone" };
const int servoMenuCount = 5;

int menuIndex = 0;
Screen prevMenu = SCR_MAIN_MENU;
int editingItem = 0;

enum EditType { EDIT_INT, EDIT_FLOAT, EDIT_BOOL };

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
      case 4: cfg.steerDeadzone = edit.intVal; break;
    }
  }
}

// ─── Display ─────────────────────────────────────────────────────────────────

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
  if (ctrlConnected || wsActive) {
    display.setTextSize(2);
    display.setCursor(0, 0);
    if (wsClientConnected) {
      display.print("WS");
    } else {
      display.print(presetNames[currentPreset]);
    }

    display.setTextSize(2);
    display.setCursor(108, 0);
    display.print(gearChar(currentGear));

    display.setTextSize(1);
    display.setCursor(0, 18);
    if (expMode)   display.print("EXP ");
    if (handbrake) display.print("BRK ");
    if (wsActive)  display.print("WS");

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

  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) {
    delay(250);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed");
  }

  // WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.printf("WebSocket server on port %d\n", WS_PORT);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  Serial.println("=== Ready ===");
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  int delta = readEncoder();
  int btn = readButton();
  if (delta != 0 || btn != 0) lastActivityTime = millis();
  BP32.update();
  updateVroom();
  webSocket.loop();

  // WS control timeout
  if (wsActive && millis() - lastWsControl > WS_CONTROL_TIMEOUT) {
    wsActive = false;
    wsThrottle = 0;
    wsSteer = 0;
  }

  // Telemetry broadcast
  if (wsClientConnected && millis() - lastTelemTime >= TELEM_INTERVAL_MS) {
    lastTelemTime = millis();
    broadcastTelemetry();
  }

  // Menu timeout
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
      applyLED();
    }
    if (r1Pressed && !lastR1 && currentPreset < presetCount - 1) {
      currentPreset++;
      cfg = presets[currentPreset];
      applyLED();
    }
    lastL1 = l1Pressed;
    lastR1 = r1Pressed;

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
      if (handbrake) {
        motorSignal = 1000;
      }
    }

    // Handbrake rumble
    if (handbrake != lastHandbrake) {
      if (handbrake) {
        if (dualshock) dualshock->playDualRumble(0, 65535, 80, 0);
      } else {
        if (dualshock) dualshock->playDualRumble(0, 0, 0, 0);
      }
      lastHandbrake = handbrake;
    }

  } else if (wsActive) {
    // ── WebSocket remote control (when no gamepad) ──────────────────────────
    if (expMode) {
      int target = constrain(map((int)wsThrottle, 0, 100, 1000, cfg.maxTiming), 1000, cfg.maxTiming);
      float stickPct = wsThrottle / 100.0f;
      smoothSignal += (stickPct > 0.02f)
                        ? (target - smoothSignal) * cfg.expAccelRate
                        : (1000   - smoothSignal) * cfg.expDecayRate;
      motorSignal = constrain((int)smoothSignal, 1000, cfg.maxTiming);
    } else {
      motorSignal = constrain(map((int)wsThrottle, 0, 100, 1000, cfg.maxTiming), 1000, cfg.maxTiming);
      smoothSignal = motorSignal;
    }
    if (motorSignal < 1000 + cfg.deadzone) motorSignal = 1000;
    // handle handbrake
    if (handbrake) {
      motorSignal = 1000;
      smoothSignal = 1000;
    }

    int wsMapped;
    if (wsSteer >= 0) {
      wsMapped = map((int)wsSteer, 0, 100, cfg.servoCenter, cfg.servoMax);
    } else {
      wsMapped = map((int)wsSteer, -100, 0, cfg.servoMin, cfg.servoCenter);
    }
    servoAngle = constrain(wsMapped, cfg.servoMin, cfg.servoMax);

    shifterAngle = gearToAngle(currentGear);

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
      if (btn == 1) { currentScreen = SCR_MAIN_MENU; menuIndex = 0; }
      updateDriveDisplay();
      break;

    case SCR_MAIN_MENU:
      menuIndex = constrain(menuIndex + delta, 0, mainMenuCount - 1);
      if (btn == 1) {
        switch (menuIndex) {
          case 0: currentScreen = SCR_MOTOR_MENU; menuIndex = 0; break;
          case 1: currentScreen = SCR_SERVO_MENU; menuIndex = 0; break;
          case 2: currentScreen = SCR_PRESET_MENU; menuIndex = currentPreset; break;
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
      if (btn == 2) { currentScreen = SCR_DRIVE; menuIndex = 0; }
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
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 0; }
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
          case 4: enterEditInt("Steer deadzone", cfg.steerDeadzone, 0, 100, 1); break;
        }
      }
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 0; }
      if (currentScreen == SCR_SERVO_MENU)
        drawScrollMenu("Servo", servoMenuItems, servoMenuCount, menuIndex);
      break;

    case SCR_PRESET_MENU:
      menuIndex = constrain(menuIndex + delta, 0, presetCount - 1);
      if (btn == 1) {
        currentPreset = menuIndex;
        cfg = presets[currentPreset];
        applyLED();
        currentScreen = SCR_DRIVE;
        menuIndex = 0;
      }
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 2; }
      if (currentScreen == SCR_PRESET_MENU)
        drawScrollMenu("Mode", presetNames, presetCount, menuIndex);
      break;

    case SCR_CTRL_MONITOR:
      if (btn == 2) { currentScreen = SCR_MAIN_MENU; menuIndex = 3; }
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
      if (btn == 1) { applyEdit(); currentScreen = prevMenu; menuIndex = editingItem; }
      if (btn == 2) { currentScreen = prevMenu; menuIndex = editingItem; }
      drawEditScreen();
      break;
  }

  delay(5);
}
