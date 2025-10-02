#include "sensorGPSUBlox.h"

sensorGPSUBlox::sensorGPSUBlox() {
}

int sensorGPSUBlox::setup(HardwareSerial& port, int baud) {
  int results = sensorGPS::setup(baud);

  Serial.println(F("Setup GPS..."));
  Serial.println(F("\t...default baud..."));
  port.begin(9600);

  // Wait for the GPS to power up
  delay(1000);

  results = 0;
  Serial.printf(F("\t...requesting new baud rate of %d...\n"), baud);
  // Send the command to change the baud rate
  for (int i = 0; i < 5; i++) {
    Serial.printf(F("\t\t...request %d...\n"), i);
    if (baud == 57600)
      port.write(commandBaudRate57600, sizeof(commandBaudRate57600));
    else if (baud == 115200)
      port.write(commandBaudRate115200, sizeof(commandBaudRate115200));
    else {
      results = 1;
      port.println(F("\tBaud rate not supported."));
    }
  }

  port.end();
  
  if (results > 0) {
    Serial.println(F("..setup GPS failed."));
    return results;
  }

  Serial.printf(F("\t...switching to baud rate of %d.\n"), baud);
  port.begin(baud);

  Serial.println(F("...setup GPS finished."));
  
  return results;
}