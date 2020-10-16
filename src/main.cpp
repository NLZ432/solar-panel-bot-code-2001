#include <Arduino.h>
#include "PanelPlacer.h"
--hi
PanelPlacer bot;

void setup() {
  bot.init();
}

void loop() {

  bot.run();

  delay(100);
}