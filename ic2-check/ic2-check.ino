#include <Wire.h>
void setup() {
  Wire.begin(12, 14);
  Serial.begin(115200);
  for(byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if(Wire.endTransmission() == 0)
      Serial.printf("Found device at 0x%X\n", i);
  }
}
void loop() {}