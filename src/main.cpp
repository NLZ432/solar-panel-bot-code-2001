#include <Arduino.h>
#include "PanelPlacer.h"
PanelPlacer bot;
void setup() {
  bot.init();
}

/**
 * hello
 * you smell
 * **/
void loop() {
 
  
  bot.run();

  delay(100);
}
