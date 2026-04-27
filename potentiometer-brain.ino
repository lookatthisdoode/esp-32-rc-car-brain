#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>
#include <WiFi.h>
#include <WebServer.h>

// Pin definitions
#define POT_PIN 34
#define MOTOR_PIN 19
#define SERVO_PIN 18
#define OLED_SDA 21
#define OLED_SCL 22

// Whole motor range in microseconds
#define LOW_TIMING 1500
#define HIGH_TIMING 2200

// WiFi
const char* ssid     = "REQUIRED";
const char* password = "REQUIRED";

// OLED
Adafruit_SSD1306 display(128, 64, &Wire, -1);

Servo motorESC;
Servo steeringServo;
WebServer server(6767);
ControllerPtr dualshock;

// Global state
int motorSignal       = LOW_TIMING;
int servoAngle        = 90;
int potValue          = 0;
bool ctrlConnected    = false;
int lxVal = 0, ryVal = 0, r2Val = 0;

// ─── Bluepad32 callbacks ──────────────────────────────────────────────────────

void onConnectedController(ControllerPtr ctl) {
  Serial.println(">>> PS4 Controller connected!");
  dualshock = ctl;
  ctrlConnected = true;
}

void onDisconnectedController(ControllerPtr ctl) {
  Serial.println(">>> PS4 Controller disconnected!");
  if (ctl == dualshock) dualshock = nullptr;
  ctrlConnected = false;
  // Safe state on disconnect
  motorSignal = LOW_TIMING;
  servoAngle  = 90;
}

// ─── OLED helpers ─────────────────────────────────────────────────────────────

void drawHBar(int x, int y, int w, int h, float pct) {
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  int fill = constrain((int)(pct * (w - 4)), 0, w - 4);
  if (fill > 0)
    display.fillRect(x + 2, y + 2, fill, h - 4, SSD1306_WHITE);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (ctrlConnected) {
    // ── Controller mode ──
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Mode: CONTROLLER");

    // Throttle bar  LOW_TIMING-HIGH_TIMING mapped to 0-1)
    display.setCursor(0, 10);
    display.print("Throttle");
    drawHBar(0, 19, 128, 8, (motorSignal - LOW_TIMING) / 700.0);

    // Steering bar (0-180 mapped to 0-1)
    display.setCursor(0, 30);
    display.print("Steering");
    drawHBar(0, 39, 128, 8, servoAngle / 180.0);
    // Center marker
    display.drawFastVLine(64, 39, 8, SSD1306_WHITE);

    // Values
    display.setCursor(0, 50);
    display.printf("M:%4dus S:%3ddeg", motorSignal, servoAngle);

  } else {
    // ── Pot mode ──
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Mode: POT");

    display.setCursor(0, 10);
    display.printf("Pot raw: %4d", potValue);

    // Motor bar
    display.setCursor(0, 22);
    display.print("Motor");
    drawHBar(0, 31, 128, 8, (motorSignal - LOW_TIMING) /700.0);

    display.setCursor(0, 42);
    display.printf("Signal: %4d us", motorSignal);

    display.setCursor(0, 54);
    display.println("BT: waiting...");
  }

  display.display();
}

// ─── HTTP server ──────────────────────────────────────────────────────────────

void handleRoot() {
  String html = R"(
<!DOCTYPE html><html>
<head>
  <title>RC Car Debug</title>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    body { font-family: monospace; background: #111; color: #0f0; padding: 20px; }
    .val { color: #fff; font-size: 1.2em; }
    .card { background: #1a1a1a; border: 1px solid #0f0; padding: 12px; margin: 8px 0; border-radius: 6px; }
    .connected { color: #0f0; } .disconnected { color: #f00; }
    .bar-wrap { background: #333; border-radius: 4px; height: 16px; margin: 4px 0; }
    .bar { background: #0f0; height: 16px; border-radius: 4px; transition: width 0.1s; }
    h1 { color: #0f0; margin-bottom: 4px; }
  </style>
  <script>
    function pct(val, min, max) { return Math.round((val-min)/(max-min)*100) + '%'; }
    function refresh() {
      fetch('/data').then(r=>r.json()).then(d=>{
        document.getElementById('mode').innerHTML = d.controller
          ? "<span class='connected'>CONTROLLER</span>"
          : "<span class='disconnected'>POT</span>";
        document.getElementById('motor').textContent = d.motor + ' us';
        document.getElementById('servo').textContent = d.servo + ' deg';
        document.getElementById('pot').textContent   = d.pot;
        document.getElementById('lx').textContent    = d.lx;
        document.getElementById('ry').textContent    = d.ry;
        document.getElementById('r2').textContent    = d.r2;
        document.getElementById('mbar').style.width  = pct(d.motor 1500,2200);
        document.getElementById('sbar').style.width  = pct(d.servo,0,180);
      });
    }
    setInterval(refresh, 100);
  </script>
</head>
<body>
  <h1>RC Car Debug</h1>
  <div class='card'>Mode: <span class='val' id='mode'>-</span></div>
  <div class='card'>
    <b>Motor</b> <span class='val' id='motor'>-</span>
    <div class='bar-wrap'><div class='bar' id='mbar' style='width:0%'></div></div>
    <b>Steering</b> <span class='val' id='servo'>-</span>
    <div class='bar-wrap'><div class='bar' id='sbar' style='width:0%'></div></div>
  </div>
  <div class='card'>
    Pot: <span class='val' id='pot'>-</span><br>
    LX:  <span class='val' id='lx'>-</span><br>
    RY:  <span class='val' id='ry'>-</span><br>
    R2:  <span class='val' id='r2'>-</span>
  </div>
</body></html>
  )";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{";
  json += "\"controller\":" + String(ctrlConnected ? "true" : "false") + ",";
  json += "\"motor\":"  + String(motorSignal) + ",";
  json += "\"servo\":"  + String(servoAngle)  + ",";
  json += "\"pot\":"    + String(potValue)    + ",";
  json += "\"lx\":"     + String(lxVal)       + ",";
  json += "\"ry\":"     + String(ryVal)       + ",";
  json += "\"r2\":"     + String(r2Val);
  json += "}";
  server.send(200, "application/json", json);
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  steeringServo.attach(SERVO_PIN, 1000, 2000);
  steeringServo.write(90);

  // ESC arming
  // Read pot FIRST and set ESC before anything else
  int initialPot = analogRead(POT_PIN);
  motorESC.attach(MOTOR_PIN, LOW_TIMING, HIGH_TIMING);
  int initialSignal = map(initialPot, 0, 4095, LOW_TIMING, HIGH_TIMING);
  initialSignal = constrain(initialSignal, LOW_TIMING, HIGH_TIMING);
  motorESC.writeMicroseconds(initialSignal);
  Serial.printf("Initial pot: %d, signal: %d\n", initialPot, initialSignal);
  Serial.println("ESC armed");

  Serial.println("=== RC Car Brain Booting ===");

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed - continuing without display");
  } else {
    Serial.println("OLED OK");
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected! http://%s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nWiFi failed - continuing without it");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  Serial.println("Bluetooth ready - waiting for PS4 controller");
  Serial.println("=== Boot complete ===");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
  potValue = analogRead(POT_PIN);
  BP32.update();
  server.handleClient();

  if (ctrlConnected && dualshock && dualshock->isConnected()) {
    lxVal = dualshock->axisX();
    ryVal = dualshock->axisRY();
    r2Val = dualshock->throttle();

    //motorSignal = map(ryVal, -512, 511, HIGH_TIMING, LOW_TIMING); // Right stick
    motorSignal = map(r2Val, 0, 1020, LOW_TIMING, HIGH_TIMING); // R2
    motorSignal = constrain(motorSignal, LOW_TIMING, HIGH_TIMING);
    servoAngle  = map(lxVal, -512, 511, 0, 180);
    servoAngle  = constrain(servoAngle, 0, 180);

    Serial.printf("CTRL | Motor: %4dus | Servo: %3ddeg | LX: %4d | RY: %4d | R2: %3d\n",
                  motorSignal, servoAngle, lxVal, ryVal, r2Val);
  } else {
    lxVal = ryVal = r2Val = 0;
    motorSignal = map(potValue, 0, 4095, LOW_TIMING, HIGH_TIMING);
    motorSignal = constrain(motorSignal, LOW_TIMING, HIGH_TIMING);
    servoAngle  = 90;

    Serial.printf("POT  | Motor: %4dus | Pot: %4d\n", motorSignal, potValue);
  }

  motorESC.writeMicroseconds(motorSignal);
  steeringServo.write(servoAngle);
  updateDisplay();
  delay(20);
}
