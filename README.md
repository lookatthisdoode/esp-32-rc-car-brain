# Arduino based brains for a lego technic RC car.

I am building lego technic RC car and since i have some arduinos laying around I decided why not make it DS4 controlled as well. Fixed little screen to it as well but thats for debugging mainly, its so fragile to place in the car for now. So I am hosting a http debug page too. 

 # Deps list
Most libraries are okay to get by name, but make sure to install [Bluepad32](https://bluepad32.readthedocs.io/en/latest/plat_arduino/) board.

```
#include  <Adafruit_GFX.h>
#include  <Adafruit_SSD1306.h>
#include  <ESP32Servo.h>
#include  <Bluepad32.h> // special board containing some BT stuff needed
#include  <WiFi.h>
#include  <WebServer.h>
```

# Todos
- create and add Lego files here
- better debug page
- somehow bundle deps
