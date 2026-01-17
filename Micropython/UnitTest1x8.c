#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_NeoPixel.h>

// Neopixel-Setup
#define NEOPIXEL_PIN D7
#define NUM_PIXELS 8
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// MCP23017-Setup
Adafruit_MCP23X17 mcp;
uint8_t lastSensorStates = 0;

void printSensorStates(uint8_t states) {
  Serial.print("sensorStates: ");
  for (int i = 7; i >= 0; i--) {
    Serial.print((states & (1 << i)) ? "1 " : "0 ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pixels.begin();
  pixels.clear();
  pixels.setBrightness(128);
  pixels.show();

  if (!mcp.begin_I2C(0x20, &Wire)) {
    Serial.println("MCP23017 nicht gefunden!");
    while (1) delay(10);
  }

  for (int i = 0; i < 8; i++) {
    mcp.pinMode(i, INPUT_PULLUP);
  }

  Serial.println("Testprogramm gestartet.");
}

void loop() {
  uint8_t sensorStates = 0;
  for (int i = 0; i < 8; i++) {
    if (mcp.digitalRead(i) == LOW) {
      sensorStates |= (1 << i);
    }
  }

  printSensorStates(sensorStates);

  if (sensorStates != lastSensorStates) {
    for (int i = 0; i < 8; i++) {
      bool currentState = (sensorStates & (1 << i));
      bool lastState = (lastSensorStates & (1 << i));
      if (currentState != lastState) {
        Serial.print("Hall-Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(currentState ? "Magnet erkannt" : "Kein Magnet");
      }
    }
    lastSensorStates = sensorStates;
  }

  for (int i = 0; i < 8; i++) {
    pixels.setPixelColor(i, (sensorStates & (1 << i)) ? pixels.Color(255, 0, 0) : pixels.Color(0, 0, 0));
  }
  pixels.show();

  delay(100);
}
