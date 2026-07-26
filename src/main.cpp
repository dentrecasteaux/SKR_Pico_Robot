#include <Arduino.h>
#include "Hardware.h"
#include "Config.h"
#include "Stepper.h"

Stepper leftStepper(HW::LEFT_EN);
Stepper rightStepper(HW::RIGHT_EN);

void setup() {
  leftStepper.begin();
  rightStepper.begin();

  Serial.begin(Config::SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  Serial.println("SKR Pico starting...");
  Serial.print("Left step pin: ");
  Serial.println(HW::LEFT_STEP);
  Serial.println("Stepper drivers initialised and disabled.");
}

void loop() {
  Serial.println("Hello from the SKR Pico");
  delay(Config::STATUS_PERIOD_MS);
}
