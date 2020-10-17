#include <Arduino.h>
#include "PanelPlacer.h"
PanelPlacer bot;
void setup() {
  bot.init();
}

void loop() {
 
  Serial.print("hello");
  bot.run();

  delay(100);
}