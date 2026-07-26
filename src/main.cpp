#include <Arduino.h>
#include "Hardware.h"
#include "Config.h"

void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  Serial.println("SKR Pico starting...");
  Serial.print("Left step pin: ");
  Serial.println(HW::LEFT_STEP);
}

void loop() {
  Serial.println("Hello from the SKR Pico");
  delay(Config::STATUS_PERIOD_MS);
}