#include "ld06.h"

LD06 ld06;

void setup() {
  Serial.begin(115200);
  ld06.init(1);
}

void loop() {
   ld06.readFullScan();
}
