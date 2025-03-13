// External libraries
#include <Arduino.h>
#include <CRC16.h>

// Included files
#include "MotorControl.h"
#include "Communication.h"
#include "ErrorHandling.h"


void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(1000000);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  // prints title with ending line break
  Serial.println("ASCII Table ~ Character Map");
}

// first visible ASCIIcharacter '!' is number 33:
int thisByte = 33;
// you can also write ASCII characters in single quotes.
// for example, '!' is the same as 33, so you could also use this:
// int thisByte = '!';

void loop() {

}
