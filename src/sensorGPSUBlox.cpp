#include "sensorGPSUBlox.h"

sensorGPSUBlox::sensorGPSUBlox() {
}

int8_t sensorGPSUBlox::setup(HardwareSerial& wire, int baud) {
  byte results = sensorGPS::setup(wire, baud);

  Serial.println(F("\tSetup GPS..."));
  wire.begin(9600);

  // Wait for the GPS to power up
  delay(1000);

  results = 0;
  Serial.printf(F("\t\t...requesting new Baud rate of %d...\n"), baud);
  // Send the command to change the baud rate
  for (int i = 0; i < 5; i++) {
    // Serial.printf(F("\t\t...request %d...\n"), i);
    if (baud == 57600)
      wire.write(commandBaudRate57600, sizeof(commandBaudRate57600));
    else if (baud == 115200)
      wire.write(commandBaudRate115200, sizeof(commandBaudRate115200));
    else {
      results = 1;
      wire.println(F("\t\tBaud rate not supported."));
    }
  }

  wire.end();
  
  if (results > 0) {
    Serial.println(F("\t..new Baud rate failed."));
    return results;
  }

  Serial.printf(F("\t\t...switching to baud rate of %d.\n"), baud);
  wire.begin(baud);

  Serial.println(F("\t...setup GPS finished."));
  
  return results;
}